/*
 * Copyright (c) 2014, ATR, Atsushi Watanabe
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the copyright holder nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef HOKUYO3D__VSSP_HPP_
#define HOKUYO3D__VSSP_HPP_

#include <boost/asio.hpp>
#include <boost/asio/system_timer.hpp>
#include <boost/array.hpp>
#include <boost/format.hpp> 
#include <boost/serialization/access.hpp>
#include <boost/archive/binary_iarchive.hpp>
#include <boost/shared_array.hpp>
#include <boost/algorithm/string.hpp>
#include <chrono>//変更9.17

#include <vector>
#include <string>

#include <hokuyo3d/vsspdefs.hpp>//vsspにすべきか？

namespace vssp
{
class VsspDriver
{
private:
  boost::asio::io_service io_service_;
  boost::asio::ip::tcp::socket socket_;
  boost::asio::system_timer timer_;
  bool closed_;
  AuxFactorArray aux_factor_;

  std::function<void(
      const vssp::Header&,
      const vssp::RangeHeader&,
      const vssp::RangeIndex&,
      const boost::shared_array<uint16_t>&,
      const boost::shared_array<vssp::XYZI>&,
      const std::chrono::system_clock::time_point&)> cb_point_;
  std::function<void(
      const vssp::Header&,
      const vssp::AuxHeader&,
      const boost::shared_array<vssp::Aux>&,
      const std::chrono::system_clock::time_point&)> cb_aux_;
  std::function<void(
      const vssp::Header&,
      const std::chrono::system_clock::time_point&)> cb_ping_;
  std::function<void(
      const vssp::Header&,
      const std::string&,
      const std::chrono::system_clock::time_point&)> cb_error_;
  std::function<void(bool)> cb_connect_;
  boost::shared_array<const double> tbl_h_;
  std::vector<boost::shared_array<const TableSincos>> tbl_v_;
  bool tbl_h_loaded_;
  bool tbl_v_loaded_;
  std::vector<bool> tbl_vn_loaded_;
  std::chrono::duration timeout_;

  boost::asio::streambuf buf_;

public:
  VsspDriver();
  
  void setTimeout(const double to);  
  void connect(const char* ip, const unsigned int port, decltype(cb_connect_) cb);
  void registerErrorCallback(decltype(cb_error_) cb);
  void registerCallback(decltype(cb_point_) cb);
  void registerAuxCallback(decltype(cb_aux_) cb);
  void registerPingCallback(decltype(cb_ping_) cb);
  void setAutoReset(const bool enable);
  [[deprecated("use setHorizontalInterlace() instead of setInterlace()")]] void setInterlace(const int itl);
  void setHorizontalInterlace(const int itl);
  void setVerticalInterlace(const int itl);
  void requestVerticalTable(const int itl = 1);  
  void requestHorizontalTable();
  void requestPing();
  void requestAuxData(const bool start = 1);
  void requestData(const bool intensity = 1, const bool start = 1);
  void receivePackets();
  bool poll();
  void spin();
  void stop(); 
  boost::asio::io_service& getIoService();


private:
  void send(const std::string cmd);
  void onTimeoutConnect(const boost::system::error_code& error);
  void onTimeout(const boost::system::error_code& error);
  void onConnect(const boost::system::error_code& error);
  void onSend(const boost::system::error_code& error, boost::shared_ptr<std::string> data);

  template <class DATA_TYPE>
  bool rangeToXYZ(
      const vssp::RangeHeader& range_header,
      const vssp::RangeHeaderV2R1& range_header_v2r1,
      const vssp::RangeIndex& range_index,
      const boost::shared_array<const uint16_t>& index,
      const boost::shared_array<vssp::XYZI>& points);
  
  void onRead(const boost::system::error_code& error);
  
};

}  // namespace vssp

#endif  // VSSP_H
