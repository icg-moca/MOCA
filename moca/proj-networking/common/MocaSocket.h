#if 1
#include <iostream>
#include <boost/array.hpp>
#include <boost/asio.hpp>
#include <boost\bind.hpp>
#include <boost/thread.hpp>
#include <boost/chrono.hpp>
#include <boost/thread/mutex.hpp>
//======================
// Socket
//======================
class Socket {
public:
	boost::shared_ptr<boost::asio::ip::tcp::socket> socket;
public:
	void setOption() {
		boost::asio::ip::tcp::no_delay option(true);
		socket->set_option(option);
	}

	void connect(boost::asio::ip::tcp::resolver::iterator &endpoint_iter, boost::system::error_code &error) {
		socket->connect(*endpoint_iter, error);
	}

	void close() {
		socket->close();
	}

public:
	bool read(void *data, size_t size) {
		boost::system::error_code error;
		size_t len = boost::asio::read(*socket, boost::asio::buffer(reinterpret_cast<char*>(data), (std::streamsize)(size)), boost::asio::transfer_exactly(size), error);
		if (error) {
			std::cout << "[socket::read] error : " << error << std::endl;
			return false;
		}
		if (size != len) {
			std::cout << "[socket::read] size error : " << size << ", " << len << std::endl;
			return false;
		}
		return true;
	}

	bool read(boost::asio::streambuf &streambuf) {
		boost::system::error_code error;
		size_t len = boost::asio::read_until(*socket, streambuf, "\n\n");
		if (error) {
			std::cout << "[socket::write] error : " << error << std::endl;
			return false;
		}
		if (streambuf.size() != len) {
			std::cout << "[socket::write] size error : " << streambuf.size() << ", " << len << std::endl;
			return false;
		}
		std::istream request_stream(&streambuf);
		std::string filename; size_t fileSize = 0;
		request_stream >> filename;
		request_stream >> fileSize;
		return true;
	}

	template< class T >
	bool read_buffer_with_size(std::vector<T> &buffer, size_t &buffer_size_bytes) {
		boost::system::error_code error;

		// read buffer size
		int buf_size = 0;
		if (!read(&buf_size, 4)) {
			return false;
		}
		// read buffer
		buffer.resize(buf_size / sizeof(T));
		if (!read(buffer.data(), buf_size)) {
			return false;
		}
		buffer_size_bytes = buf_size;
		return true;
	}

	template< class T >
	bool read_buffer(std::vector<T> &buffer) {
		boost::system::error_code error;

		// read buffer size
		int buf_size = 0;
		if (!read(&buf_size, 4)) {
			return false;
		}
		// read buffer
		buffer.resize(buf_size / sizeof(T));
		if (!read(buffer.data(), buf_size)) {
			return false;
		}
		return true;
	}
	
	bool read_string(std::string &str) {
		boost::system::error_code error;

		// read buffer size
		int buf_size = -1;
		if (!read(&buf_size, 4)) {
			return false;
		}

		// read buffer
		if (!read(&str, buf_size)) {
			return false;
		}

		return true;
	}

	bool write(const void *data, size_t size) {
		boost::system::error_code error;
		size_t len = boost::asio::write(*socket, boost::asio::buffer((char*)(data), (std::streamsize)(size)), boost::asio::transfer_exactly(size), error);
		if (error) {
			std::cout << "[socket::write] error : " << error << std::endl;
			return false;
		}
		if (size != len) {
			std::cout << "[socket::write] size error : " << size << ", " << len << std::endl;
			return false;
		}
		return true;
	}

	bool write(boost::asio::streambuf &streambuf) {
		size_t checksum = streambuf.size();
		boost::system::error_code error;
		size_t len = boost::asio::write(*socket, streambuf, boost::asio::transfer_exactly(checksum), error);
		if (error) {
			std::cout << "[socket::write] error : " << error << std::endl;
			return false;
		}
		if (checksum != len) {
			std::cout << "[socket::write] size error : " << streambuf.size() << ", " << len << std::endl;
			return false;
		}
		return true;
	}

	template< class T >
	bool write_buffer(std::vector<T> &buffer) {
		boost::system::error_code error;

		// write buffer size
		int buf_size = buffer.size() * sizeof(T);
		if (!write(&buf_size, 4)) {
			return false;
		}

		// write buffer
		if (!write(buffer.data(), buf_size)) {
			return false;
		}

		return true;
	}

	bool write_string(const char *str) {
		boost::system::error_code error;

		// write buffer size
		int buf_size = strlen(str) + 1;
		if (!write(&buf_size, 4)) {
			return false;
		}

		// write buffer
		if (!write(str, buf_size)) {
			return false;
		}

		return true;
	}
};

#endif