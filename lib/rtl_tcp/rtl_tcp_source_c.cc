/* -*- c++ -*- */
/*
 * Copyright 2012 Dimitri Stolnikov <horiz0n@gmx.net>
 *
 * GNU Radio is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 3, or (at your option)
 * any later version.
 *
 * GNU Radio is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with GNU Radio; see the file COPYING.  If not, write to
 * the Free Software Foundation, Inc., 51 Franklin Street,
 * Boston, MA 02110-1301, USA.
 */

#include <fstream>
#include <string>
#include <sstream>

#include <boost/assign.hpp>
#include <boost/algorithm/string.hpp>

#include <gnuradio/io_signature.h>
#include <gnuradio/blocks/deinterleave.h>
#include <gnuradio/blocks/float_to_complex.h>

#include "rtl_tcp_source_c.h"

#include "arg_helpers.h"
#include <sys/ioctl.h>

/* copied from rtl sdr code */
typedef struct { /* structure size must be multiple of 2 bytes */
  char magic[4];
  uint32_t tuner_type;
  uint32_t tuner_gain_count;
} dongle_info_t;

#define USE_SELECT    1  // non-blocking receive on all platforms
#define USE_RCV_TIMEO 0  // non-blocking receive on all but Cygwin

#define BYTES_PER_SAMPLE  2 // rtl device delivers 8 bit unsigned IQ data

using namespace boost::assign;

static std::string get_tuner_name( enum rtlsdr_tuner tuner_type )
{
  if ( RTLSDR_TUNER_E4000 == tuner_type )
    return "E4000";
  else if ( RTLSDR_TUNER_FC0012 == tuner_type )
    return "FC0012";
  else if ( RTLSDR_TUNER_FC0013 == tuner_type )
    return "FC0013";
  else if ( RTLSDR_TUNER_FC2580 == tuner_type )
    return "FC2580";
  else if ( RTLSDR_TUNER_R820T == tuner_type )
    return "R820T";
  else if ( RTLSDR_TUNER_R828D == tuner_type )
    return "R828D";
  else
    return "Unknown";
}

rtl_tcp_source_c_sptr make_rtl_tcp_source_c(const std::string &args)
{
  return gnuradio::get_initial_sptr(new rtl_tcp_source_c(args));
}

static int is_error( int perr )
{
  // Compare error to posix error code; return nonzero if match.
#if defined(USING_WINSOCK)
#define ENOPROTOOPT 109
  // All codes to be checked for must be defined below
  int werr = WSAGetLastError();
  switch( werr ) {
  case WSAETIMEDOUT:
    return( perr == EAGAIN );
  case WSAENOPROTOOPT:
    return( perr == ENOPROTOOPT );
  default:
    fprintf(stderr,"rtl_tcp_source_f: unknown error %d WS err %d \n", perr, werr );
    throw std::runtime_error("internal error");
  }
  return 0;
#else
  return( perr == errno );
#endif
}


static void report_error( const char *msg1, const char *msg2 )
{
  // Deal with errors, both posix and winsock
#if defined(USING_WINSOCK)
  int werr = WSAGetLastError();
  fprintf(stderr, "%s: winsock error %d\n", msg1, werr );
#else
  perror(msg1);
#endif
  if( msg2 != NULL )
    throw std::runtime_error(msg2);
  return;
}

static const int MIN_IN = 0;	// mininum number of input streams
static const int MAX_IN = 0;	// maximum number of input streams
static const int MIN_OUT = 1;	// minimum number of output streams
static const int MAX_OUT = 1;	// maximum number of output streams


rtl_tcp_source_c::rtl_tcp_source_c(const std::string &args) :
  gr::sync_block("rtl_tcp_source_c",
                 gr::io_signature::make(MIN_IN, MAX_IN, sizeof (gr_complex)),
                 gr::io_signature::make(MIN_OUT, MAX_OUT, sizeof (gr_complex))),
  _no_tuner(false),
  _auto_gain(false),
  _if_gain(0),
  _running(false),
  _start(true)
{
  std::string host = "127.0.0.1";
  unsigned short port = 1234;
  int payload_size = 16384;
  unsigned int direct_samp = 0, offset_tune = 0;

  _freq = 0;
  _rate = 0;
  _gain = 0;
  _corr = 0;

  dict_t dict = params_to_dict(args);

  if (dict.count("rtl_tcp")) {
    std::vector< std::string > tokens;
    boost::algorithm::split( tokens, dict["rtl_tcp"], boost::is_any_of(":") );

    if ( tokens[0].length() && (tokens.size() == 1 || tokens.size() == 2 ) )
      host = tokens[0];

    if ( tokens.size() == 2 ) // port given
      port = boost::lexical_cast< unsigned short >( tokens[1] );
  }

  if (dict.count("psize"))
    payload_size = boost::lexical_cast< int >( dict["psize"] );

  if (dict.count("direct_samp"))
    direct_samp = boost::lexical_cast< unsigned int >( dict["direct_samp"] );

  if (dict.count("offset_tune"))
    offset_tune = boost::lexical_cast< unsigned int >( dict["offset_tune"] );

  if (!host.length())
    host = "127.0.0.1";

  if (0 == port)
    port = 1234;

  if (payload_size <= 0)
    payload_size = 16384;

  int ret = 0;
#if defined(USING_WINSOCK) // for Windows (with MinGW)
  // initialize winsock DLL
  WSADATA wsaData;
  int iResult = WSAStartup( MAKEWORD(2,2), &wsaData );
  if( iResult != NO_ERROR ) {
    report_error( "rtl_tcp_source_f WSAStartup", "can't open socket" );
  }
#endif

  // Set up the address stucture for the source address and port numbers
  // Get the source IP address from the host name
  struct addrinfo *ip_src;      // store the source IP address to use
  struct addrinfo hints;
  memset( (void*)&hints, 0, sizeof(hints) );
  hints.ai_family = AF_UNSPEC;
  hints.ai_socktype = SOCK_STREAM;
  hints.ai_protocol = IPPROTO_TCP;
  hints.ai_flags = AI_PASSIVE;
  char port_str[12];
  sprintf( port_str, "%d", port );

  // FIXME leaks if report_error throws below
  ret = getaddrinfo( host.c_str(), port_str, &hints, &ip_src );
  if( ret != 0 )
    report_error("rtl_tcp_source_f/getaddrinfo",
                 "can't initialize source socket" );

  // FIXME leaks if report_error throws below
  d_temp_buff = new unsigned short[payload_size];   // allow it to hold up to payload_size bytes

  // create a lookup table for gr_complex values
  for (unsigned int i = 0; i <= 0xffff; i++) {
#ifdef BOOST_LITTLE_ENDIAN
    _lut.push_back( gr_complex( (float(i & 0xff) - 127.4f) * (1.0f/128.0f),
                                (float(i >> 8) - 127.4f) * (1.0f/128.0f) ) );
#else // BOOST_BIG_ENDIAN
    _lut.push_back( gr_complex( (float(i >> 8) - 127.4f) * (1.0f/128.0f),
                                (float(i & 0xff) - 127.4f) * (1.0f/128.0f) ) );
#endif
  }

  // create socket
  d_socket = socket(ip_src->ai_family, ip_src->ai_socktype,
                    ip_src->ai_protocol);

  // Turn on reuse address
  int opt_val = 1;
  if(setsockopt(d_socket, SOL_SOCKET, SO_REUSEADDR, (optval_t)&opt_val, sizeof(int)) == -1) {
    report_error("SO_REUSEADDR","can't set socket option SO_REUSEADDR");
  }

  // Don't wait when shutting down
  linger lngr;
  lngr.l_onoff  = 1;
  lngr.l_linger = 0;
  if(setsockopt(d_socket, SOL_SOCKET, SO_LINGER, (optval_t)&lngr, sizeof(linger)) == -1) {
    if( !is_error(ENOPROTOOPT) ) {  // no SO_LINGER for SOCK_DGRAM on Windows
      report_error("SO_LINGER","can't set socket option SO_LINGER");
    }
  }

#if USE_RCV_TIMEO
  // Set a timeout on the receive function to not block indefinitely
  // This value can (and probably should) be changed
  // Ignored on Cygwin
#if defined(USING_WINSOCK)
  DWORD timeout = 1000;  // milliseconds
#else
  timeval timeout;
  timeout.tv_sec = 1;
  timeout.tv_usec = 0;
#endif
  if(setsockopt(d_socket, SOL_SOCKET, SO_RCVTIMEO, (optval_t)&timeout, sizeof(timeout)) == -1) {
    report_error("SO_RCVTIMEO","can't set socket option SO_RCVTIMEO");
  }
#endif // USE_RCV_TIMEO

  while(connect(d_socket, ip_src->ai_addr, ip_src->ai_addrlen) != 0);
  freeaddrinfo(ip_src);

  int flag = 1;
  setsockopt(d_socket, IPPROTO_TCP, TCP_NODELAY, (char *)&flag,sizeof(flag));

  dongle_info_t dongle_info;
  ret = recv(d_socket, (char*)&dongle_info, sizeof(dongle_info), 0);
  if (sizeof(dongle_info) != ret)
    fprintf(stderr,"failed to read dongle info\n");

  d_tuner_type = RTLSDR_TUNER_UNKNOWN;
  d_tuner_gain_count = 0;
  d_tuner_if_gain_count = 0;

  if (memcmp(dongle_info.magic, "RTL0", 4) == 0)
  {
    d_tuner_type = ntohl(dongle_info.tuner_type);
    d_tuner_gain_count = ntohl(dongle_info.tuner_gain_count);
    if ( RTLSDR_TUNER_E4000 == d_tuner_type )
      d_tuner_if_gain_count = 53;
  }



  if ( get_tuner_type() != RTLSDR_TUNER_UNKNOWN )
  {
    std::cerr << "The RTL TCP server reports a "
              << get_tuner_name( get_tuner_type() )
              << " tuner with "
              << get_tuner_gain_count() << " RF and "
              << get_tuner_if_gain_count() << " IF gains."
              << std::endl;
  }

  set_gain_mode(false); /* enable manual gain mode by default */

  set_direct_sampling(direct_samp);
  if (direct_samp) {
    _no_tuner = true;
  }

  set_offset_tuning(offset_tune);
}

rtl_tcp_source_c::~rtl_tcp_source_c()
{
  delete [] d_temp_buff;

  if (d_socket != -1){
    shutdown(d_socket, SHUT_RDWR);
#if defined(USING_WINSOCK)
    closesocket(d_socket);
#else
    ::close(d_socket);
#endif
    d_socket = -1;
  }

#if defined(USING_WINSOCK) // for Windows (with MinGW)
  // free winsock resources
  WSACleanup();
#endif
}

std::string rtl_tcp_source_c::name()
{
  return "RTL TCP Client";
}

std::vector<std::string> rtl_tcp_source_c::get_devices( bool fake )
{
  std::vector<std::string> devices;

  if ( fake )
  {
    std::string args = "rtl_tcp=localhost:1234";
    args += ",label='RTL-SDR Spectrum Server'";
    devices.push_back( args );
  }

  return devices;
}

size_t rtl_tcp_source_c::get_num_channels( void )
{
  return 1;
}

osmosdr::meta_range_t rtl_tcp_source_c::get_sample_rates( void )
{
  osmosdr::meta_range_t range;

  range += osmosdr::range_t( 250000 ); // known to work
  range += osmosdr::range_t( 1000000 ); // known to work
  range += osmosdr::range_t( 1024000 ); // known to work
  range += osmosdr::range_t( 1800000 ); // known to work
  range += osmosdr::range_t( 1920000 ); // known to work
  range += osmosdr::range_t( 2000000 ); // known to work
  range += osmosdr::range_t( 2048000 ); // known to work
  range += osmosdr::range_t( 2400000 ); // known to work
  range += osmosdr::range_t( 2560000 ); // known to work
//  range += osmosdr::range_t( 2600000 ); // may work
//  range += osmosdr::range_t( 2800000 ); // may work
//  range += osmosdr::range_t( 3000000 ); // may work
//  range += osmosdr::range_t( 3200000 ); // max rate

  return range;
}

double rtl_tcp_source_c::set_sample_rate( double rate )
{
  set_sample_rate( int(rate) );

  _rate = rate;

  return get_sample_rate();
}

double rtl_tcp_source_c::get_sample_rate( void )
{
  return _rate;
}

osmosdr::freq_range_t rtl_tcp_source_c::get_freq_range( size_t chan )
{
  osmosdr::freq_range_t range;

  if (_no_tuner) {
    range += osmosdr::range_t( 0, double(28.8e6) ); // as far as we know
    return range;
  }

  enum rtlsdr_tuner tuner = get_tuner_type();

  if ( tuner == RTLSDR_TUNER_E4000 ) {
    /* there is a (temperature dependent) gap between 1100 to 1250 MHz */
    range += osmosdr::range_t( 52e6, 2.2e9 );
  } else if ( tuner == RTLSDR_TUNER_FC0012 ) {
    range += osmosdr::range_t( 22e6, 948e6 );
  } else if ( tuner == RTLSDR_TUNER_FC0013 ) {
    range += osmosdr::range_t( 22e6, 1.1e9 );
  } else if ( tuner == RTLSDR_TUNER_FC2580 ) {
    range += osmosdr::range_t( 146e6, 308e6 );
    range += osmosdr::range_t( 438e6, 924e6 );
  } else if ( tuner == RTLSDR_TUNER_R820T ) {
    range += osmosdr::range_t( 24e6, 1766e6 );
  } else if ( tuner == RTLSDR_TUNER_R828D ) {
    range += osmosdr::range_t( 24e6, 1766e6 );
  } else {
    range += osmosdr::range_t( 52e6, 2.2e9 ); // assume E4000 tuner
  }

  return range;
}

double rtl_tcp_source_c::set_center_freq( double freq, size_t chan )
{
  set_freq( int(freq) );

  _freq = freq;

  return get_center_freq(chan);
}

double rtl_tcp_source_c::get_center_freq( size_t chan )
{
  return _freq;
}

double rtl_tcp_source_c::set_freq_corr( double ppm, size_t chan )
{
  set_freq_corr( int(ppm) );

  _corr = ppm;

  return get_freq_corr( chan );
}

double rtl_tcp_source_c::get_freq_corr( size_t chan )
{
  return _corr;
}

std::vector<std::string> rtl_tcp_source_c::get_gain_names( size_t chan )
{
  std::vector< std::string > names;

  names += "LNA";

  if ( get_tuner_type() == RTLSDR_TUNER_E4000 ) {
    names += "IF";
  }

  return names;
}

osmosdr::gain_range_t rtl_tcp_source_c::get_gain_range( size_t chan )
{
  osmosdr::gain_range_t range;

  /* the following gain values have been copied from librtlsdr */

  /* all gain values are expressed in tenths of a dB */
  const int e4k_gains[] = { -10, 15, 40, 65, 90, 115, 140, 165, 190, 215,
                            240, 290, 340, 420 };
  const int fc0012_gains[] = { -99, -40, 71, 179, 192 };
  const int fc0013_gains[] = { -99, -73, -65, -63, -60, -58, -54, 58, 61,
                               63, 65, 67, 68, 70, 71, 179, 181, 182,
                               184, 186, 188, 191, 197 };
  const int fc2580_gains[] = { 0 /* no gain values */ };
  const int r820t_gains[] = { 0, 9, 14, 27, 37, 77, 87, 125, 144, 157,
                              166, 197, 207, 229, 254, 280, 297, 328,
                              338, 364, 372, 386, 402, 421, 434, 439,
                              445, 480, 496 };
  const int unknown_gains[] = { 0 /* no gain values */ };

  const int *ptr = NULL;
  int len = 0;

  switch (get_tuner_type())
  {
  case RTLSDR_TUNER_E4000:
    ptr = e4k_gains; len = sizeof(e4k_gains);
    break;
  case RTLSDR_TUNER_FC0012:
    ptr = fc0012_gains; len = sizeof(fc0012_gains);
    break;
  case RTLSDR_TUNER_FC0013:
    ptr = fc0013_gains; len = sizeof(fc0013_gains);
    break;
  case RTLSDR_TUNER_FC2580:
    ptr = fc2580_gains; len = sizeof(fc2580_gains);
    break;
  case RTLSDR_TUNER_R820T:
    ptr = r820t_gains; len = sizeof(r820t_gains);
    break;
  default:
    ptr = unknown_gains; len = sizeof(unknown_gains);
    break;
  }

  if ( ptr != NULL && len > 0 )
  {
    for (int i = 0; i < int(len / sizeof(int)); i++)
      range += osmosdr::range_t( ptr[i] / 10.0f );
  }

  return range;
}

osmosdr::gain_range_t rtl_tcp_source_c::get_gain_range( const std::string & name, size_t chan )
{
  if ( "IF" == name ) {
    if ( get_tuner_type() == RTLSDR_TUNER_E4000 ) {
      return osmosdr::gain_range_t(3, 56, 1);
    } else {
      return osmosdr::gain_range_t();
    }
  }

  return get_gain_range( chan );
}

bool rtl_tcp_source_c::set_gain_mode( bool automatic, size_t chan )
{
  set_gain_mode(int(!automatic));
  set_agc_mode(automatic);

  _auto_gain = automatic;

  return get_gain_mode(chan);
}

bool rtl_tcp_source_c::get_gain_mode( size_t chan )
{
  return _auto_gain;
}

double rtl_tcp_source_c::set_gain( double gain, size_t chan )
{
  osmosdr::gain_range_t gains = rtl_tcp_source_c::get_gain_range( chan );

  set_gain( int(gains.clip(gain) * 10.0) );

  _gain = gain;

  return get_gain(chan);
}

double rtl_tcp_source_c::set_gain( double gain, const std::string & name, size_t chan )
{
  if ( "IF" == name ) {
    return set_if_gain( gain, chan );
  }

  return set_gain( gain, chan );
}

double rtl_tcp_source_c::get_gain( size_t chan )
{
  return 0;
}

double rtl_tcp_source_c::get_gain( const std::string & name, size_t chan )
{
  if ( "IF" == name ) {
    return _if_gain;
  }

  return get_gain( chan );
}

double rtl_tcp_source_c::set_if_gain(double gain, size_t chan)
{
  if ( get_tuner_type() != RTLSDR_TUNER_E4000 ) {
    _if_gain = 0;
    return _if_gain;
  }

  std::vector< osmosdr::gain_range_t > if_gains;

  if_gains += osmosdr::gain_range_t(-3, 6, 9);
  if_gains += osmosdr::gain_range_t(0, 9, 3);
  if_gains += osmosdr::gain_range_t(0, 9, 3);
  if_gains += osmosdr::gain_range_t(0, 2, 1);
  if_gains += osmosdr::gain_range_t(3, 15, 3);
  if_gains += osmosdr::gain_range_t(3, 15, 3);

  std::map< int, double > gains;

  /* initialize with min gains */
  for (unsigned int i = 0; i < if_gains.size(); i++) {
    gains[ i + 1 ] = if_gains[ i ].start();
  }

  for (int i = if_gains.size() - 1; i >= 0; i--) {
    osmosdr::gain_range_t range = if_gains[ i ];

    double error = gain;

    for( double g = range.start(); g <= range.stop(); g += range.step() ) {

      double sum = 0;
      for (int j = 0; j < int(gains.size()); j++) {
        if ( i == j )
          sum += g;
        else
          sum += gains[ j + 1 ];
      }

      double err = abs(gain - sum);
      if (err < error) {
        error = err;
        gains[ i + 1 ] = g;
      }
    }
  }
#if 0
  std::cerr << gain << " => "; double sum = 0;
  for (unsigned int i = 0; i < gains.size(); i++) {
    sum += gains[ i + 1 ];
    std::cerr << gains[ i + 1 ] << " ";
  }
  std::cerr << " = " << sum << std::endl;
#endif
  for (unsigned int stage = 1; stage <= gains.size(); stage++) {
    set_if_gain(stage, int(gains[ stage ] * 10.0));
  }

  _if_gain = gain;
  return gain;
}

std::vector< std::string > rtl_tcp_source_c::get_antennas( size_t chan )
{
  std::vector< std::string > antennas;

  antennas += get_antenna(chan);

  return antennas;
}

std::string rtl_tcp_source_c::set_antenna( const std::string & antenna, size_t chan )
{
  return get_antenna(chan);
}

std::string rtl_tcp_source_c::get_antenna( size_t chan )
{
  return "RX";
}

bool rtl_tcp_source_c::start()
{
  _running = true;
  _start = true;

  return true;
}

bool rtl_tcp_source_c::stop()
{
  _running = false;

  return true;
}


int rtl_tcp_source_c::work (int noutput_items,
                            gr_vector_const_void_star &input_items,
                            gr_vector_void_star &output_items)
{
  gr_complex *out = (gr_complex *) output_items[0];
  ssize_t r = 0;

  int receivedbytes = 0;
/*
  int bytesleft = noutput_items*BYTES_PER_SAMPLE;
  int index = 0;
*/

  if (!_running) {
    fprintf(stderr, "not running\n");
    return WORK_DONE;
  }

/*
  while(bytesleft > 0) {
    receivedbytes = recv(d_socket, (char*)&d_temp_buff[index], bytesleft, 0);

    if(receivedbytes == -1 && !is_error(EAGAIN)){
      fprintf(stderr, "socket error\n");
      return -1;
    }
    bytesleft -= receivedbytes;
    index += receivedbytes;
  }
  r = noutput_items;
*/

  receivedbytes = recv(d_socket, (char*)d_temp_buff, (noutput_items*BYTES_PER_SAMPLE), 0);

  if(receivedbytes == -1 && !is_error(EAGAIN)){
    fprintf(stderr, "socket error\n");
    return -1;
  }

  r = receivedbytes/BYTES_PER_SAMPLE;

  //fprintf(stderr,"receivedbytes: %d noutput_items: %d r: %d\n",receivedbytes, noutput_items, r);

  for(int i=0; i<r; ++i) {
    out[i] = _lut[ d_temp_buff[i] ];
  }

  return r;
}

#ifdef _WIN32
#define __attribute__(x)
#pragma pack(push, 1)
#endif
struct command{
  unsigned char cmd;
  unsigned int param;
}__attribute__((packed));
#ifdef _WIN32
#pragma pack(pop)
#endif


void rtl_tcp_source_c::set_freq(int freq)
{
  struct command cmd = { 0x01, htonl(freq) };
  send(d_socket, (const char*)&cmd, sizeof(cmd), 0);
}

void rtl_tcp_source_c::set_sample_rate(int sample_rate)
{
  struct command cmd = { 0x02, htonl(sample_rate) };
  send(d_socket, (const char*)&cmd, sizeof(cmd), 0);
}

void rtl_tcp_source_c::set_gain_mode(int manual)
{
  struct command cmd = { 0x03, htonl(manual) };
  send(d_socket, (const char*)&cmd, sizeof(cmd), 0);
}

void rtl_tcp_source_c::set_gain(int gain)
{
  struct command cmd = { 0x04, htonl(gain) };
  send(d_socket, (const char*)&cmd, sizeof(cmd), 0);
}

void rtl_tcp_source_c::set_freq_corr(int ppm)
{
  struct command cmd = { 0x05, htonl(ppm) };
  send(d_socket, (const char*)&cmd, sizeof(cmd), 0);
}

void rtl_tcp_source_c::set_if_gain(int stage, int gain)
{
  uint32_t params = stage << 16 | (gain & 0xffff);
  struct command cmd = { 0x06, htonl(params) };
  send(d_socket, (const char*)&cmd, sizeof(cmd), 0);
}

void rtl_tcp_source_c::set_agc_mode(int on)
{
  struct command cmd = { 0x08, htonl(on) };
  send(d_socket, (const char*)&cmd, sizeof(cmd), 0);
}

void rtl_tcp_source_c::set_direct_sampling(int on)
{
  struct command cmd = { 0x09, htonl(on) };
  send(d_socket, (const char*)&cmd, sizeof(cmd), 0);
}

void rtl_tcp_source_c::set_offset_tuning(int on)
{
  struct command cmd = { 0x0a, htonl(on) };
  send(d_socket, (const char*)&cmd, sizeof(cmd), 0);
}
