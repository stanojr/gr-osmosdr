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
#ifndef RTL_TCP_SOURCE_C_H
#define RTL_TCP_SOURCE_C_H

#include <gnuradio/hier_block2.h>
#include <gnuradio/sync_block.h>

#include "source_iface.h"

#if defined(_WIN32)
// if not posix, assume winsock
#pragma comment(lib, "ws2_32.lib")
#define USING_WINSOCK
#include <winsock2.h>
#include <ws2tcpip.h>
#define SHUT_RDWR 2
typedef char* optval_t;
#else
#include <netdb.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <arpa/inet.h>
typedef void* optval_t;
#endif

#define ssize_t int

class rtl_tcp_source_c;

typedef boost::shared_ptr< rtl_tcp_source_c > rtl_tcp_source_c_sptr;

rtl_tcp_source_c_sptr make_rtl_tcp_source_c( const std::string & args = "" );

/* copied from rtl sdr */
enum rtlsdr_tuner {
  RTLSDR_TUNER_UNKNOWN = 0,
  RTLSDR_TUNER_E4000,
  RTLSDR_TUNER_FC0012,
  RTLSDR_TUNER_FC0013,
  RTLSDR_TUNER_FC2580,
  RTLSDR_TUNER_R820T,
  RTLSDR_TUNER_R828D
};

class rtl_tcp_source_c :
    public gr::sync_block,
    public source_iface
{
private:
  friend rtl_tcp_source_c_sptr make_rtl_tcp_source_c(const std::string &args);

  rtl_tcp_source_c(const std::string &args);

public:
  ~rtl_tcp_source_c();

  std::string name();

  enum rtlsdr_tuner get_tuner_type() { return (enum rtlsdr_tuner) d_tuner_type; }
  unsigned int get_tuner_gain_count() { return d_tuner_gain_count; }
  unsigned int get_tuner_if_gain_count() { return d_tuner_if_gain_count; }

  int work(int noutput_items,
           gr_vector_const_void_star &input_items,
           gr_vector_void_star &output_items);

  void set_freq(int freq);
  void set_sample_rate(int sample_rate);
  void set_gain_mode(int manual);
  void set_gain(int gain);
  void set_freq_corr(int ppm);
  void set_if_gain(int stage, int gain);
  void set_agc_mode(int on);
  void set_direct_sampling(int on);
  void set_offset_tuning(int on);


  static std::vector< std::string > get_devices( bool fake = false );

  size_t get_num_channels( void );

  osmosdr::meta_range_t get_sample_rates( void );
  double set_sample_rate( double rate );
  double get_sample_rate( void );

  osmosdr::freq_range_t get_freq_range( size_t chan = 0 );
  double set_center_freq( double freq, size_t chan = 0 );
  double get_center_freq( size_t chan = 0 );
  double set_freq_corr( double ppm, size_t chan = 0 );
  double get_freq_corr( size_t chan = 0 );

  std::vector<std::string> get_gain_names( size_t chan = 0 );
  osmosdr::gain_range_t get_gain_range( size_t chan = 0 );
  osmosdr::gain_range_t get_gain_range( const std::string & name, size_t chan = 0 );
  bool set_gain_mode( bool automatic, size_t chan = 0 );
  bool get_gain_mode( size_t chan = 0 );
  double set_gain( double gain, size_t chan = 0 );
  double set_gain( double gain, const std::string & name, size_t chan = 0 );
  double get_gain( size_t chan = 0 );
  double get_gain( const std::string & name, size_t chan = 0 );

  double set_if_gain( double gain, size_t chan = 0 );

  std::vector< std::string > get_antennas( size_t chan = 0 );
  std::string set_antenna( const std::string & antenna, size_t chan = 0 );
  std::string get_antenna( size_t chan = 0 );

protected:
  bool start();
  bool stop();

private:
  double _freq, _rate, _gain, _corr;
  bool _no_tuner;
  bool _auto_gain;
  double _if_gain;
  bool _running;
  bool _start;

  size_t        d_itemsize;
  bool          d_eof;           // zero-length packet is EOF
  bool          d_wait;          // wait if data if not immediately available
  int           d_socket;        // handle to socket
  unsigned short *d_temp_buff;    // hold buffer between calls
  size_t        d_temp_offset;   // point to temp buffer location offset
  std::vector<gr_complex> _lut;

  unsigned int d_tuner_type;
  unsigned int d_tuner_gain_count;
  unsigned int d_tuner_if_gain_count;


};

#endif // RTL_TCP_SOURCE_C_H
