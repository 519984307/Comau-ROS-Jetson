/**
 * @file comau_tcp_interface.cpp
 * @author LMS ()
 * @brief
 * @version 0.1
 * @date 15-02-2020
 *
 * @copyright Copyright (c) 2020
 *
 */

#include <comau_tcp_interface/comau_tcp_interface.h>

using namespace comau_tcp_interface::utils;
using namespace boost::asio::ip;

namespace comau_tcp_interface {

ComauTcpConnection::ComauTcpConnection(const ComauTcpInterfaceParameters& params) : params_(params) {
  connectToServer();
}

ComauTcpConnection::~ComauTcpConnection() {
  if (socket_ptr_) {
    socket_ptr_->close();
  }
  if (io_service_ptr_) {
    io_service_ptr_->reset();
  }
}

void ComauTcpConnection::connectToServer() {
  ROS_INFO_STREAM("["<<params_.log_tag << "] Connecting to " << params_.server_ip_address << ":" << params_.server_port);
  try {
    io_service_ptr_.reset(new boost::asio::io_service());
    tcp::resolver resolver(*io_service_ptr_);
    tcp::resolver::query query(params_.server_ip_address, params_.server_port);
    tcp::resolver::iterator endpoint_iterator = resolver.resolve(query);

    socket_ptr_.reset(new tcp::socket(*io_service_ptr_));
    socket_ptr_->open(tcp::v4());

    boost::asio::connect(*socket_ptr_, endpoint_iterator);

  } catch (const boost::system::system_error &e) {
    // leave the treatment to the initialize function of each client class
    throw e;
  }
}

void ComauTcpConnection::read(std::vector<uint8_t>& buf, const size_t buf_len, size_t& read) {
  boost::system::error_code error;
  read = socket_ptr_->read_some(boost::asio::buffer(buf), error);
  ROS_DEBUG_STREAM(params_.log_tag << "Read from socket " << read << " bytes");
  if (error) {
    throw boost::system::system_error(error);
  }
}

void ComauTcpConnection::write(const std::vector<uint8_t>& buf, const size_t buf_len, size_t& written) {
  boost::system::error_code error;
  written = socket_ptr_->write_some(boost::asio::buffer(buf), error);

  if (error) {
    throw boost::system::system_error(error);
  }
}

} // namespace comau_tcp_interface
