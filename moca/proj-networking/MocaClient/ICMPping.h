#pragma once
#ifndef __ICMP_PING_H
#define __ICMP_PING_H
#include <boost/array.hpp>
#include <boost/asio.hpp>
#include <boost\bind.hpp>
#include <boost/thread.hpp>
#include <boost/chrono.hpp>
#include <boost/thread/mutex.hpp>
#include <IP_v4.h>
//======================
// ICMP Ping
//======================
const int											MEASURETIMES = 400;
#define BOOST_ASIO_WINDOWS
using boost::asio::ip::icmp;
using boost::asio::deadline_timer;
namespace posix_time = boost::posix_time;
class Pinger {
public:
	icmp::resolver resolver;
	icmp::endpoint destination;
	icmp::socket socket;
	deadline_timer timer;
	unsigned short sequence_number;
	posix_time::ptime time_sent;
	boost::asio::streambuf reply_buffer;
	std::size_t num_replies;

	double latency;
	int c;
public:
	Pinger(boost::asio::io_service& io_service, const char* dest, int count)
		: resolver(io_service), socket(io_service, icmp::v4()),
		timer(io_service), sequence_number(0), num_replies(0), latency(0.0)
	{
		icmp::resolver::query query(icmp::v4(), dest, "");
		destination = *resolver.resolve(query);
	}

	void start_send(double &deltaT)
	{
		std::string body("\"Hello!\" from Asio ping.");

		// Create an ICMP header for an echo request.
		icmp_header echo_request;
		echo_request.type(icmp_header::echo_request);
		echo_request.code(0);
		echo_request.identifier(get_identifier());
		echo_request.sequence_number(++sequence_number);
		compute_checksum(echo_request, body.begin(), body.end());

		// Encode the request packet.
		boost::asio::streambuf request_buffer;
		std::ostream os(&request_buffer);
		os << echo_request << body;

		// Send the request.
		time_sent = posix_time::microsec_clock::universal_time();
		socket.send_to(request_buffer.data(), destination);

		// Wait up to five seconds for a reply.
		//num_replies = 0;
		//timer.expires_at(time_sent + posix_time::seconds(5));
		//timer.async_wait(boost::bind(&Pinger::handle_timeout, this));

		// Discard any data already in the buffer.
		reply_buffer.consume(reply_buffer.size());

		// Wait for a reply. We prepare the buffer to receive up to 64KB.
		size_t len = socket.receive(reply_buffer.prepare(65536));
		reply_buffer.commit(len);
		// Decode the reply packet.
		std::istream is(&reply_buffer);
		ipv4_header ipv4_hdr;
		icmp_header icmp_hdr;
		is >> ipv4_hdr >> icmp_hdr;

		// We can receive all ICMP packets received by the host, so we need to
		// filter out only the echo replies that match the our identifier and
		// expected sequence number.
		if (is && icmp_hdr.type() == icmp_header::echo_reply
			&& icmp_hdr.identifier() == get_identifier()
			&& icmp_hdr.sequence_number() == sequence_number)
		{
			// If this is the first reply, interrupt the five second timeout.
			if (num_replies++ == 0) {
				timer.cancel();
			}

			// Print out some information about the reply packet.
			posix_time::ptime now = posix_time::microsec_clock::universal_time();
			//std::cout << len - ipv4_hdr.header_length()
			//	<< " bytes from " << ipv4_hdr.source_address()
			//	<< ": icmp_seq=" << icmp_hdr.sequence_number()
			//	<< ", ttl=" << ipv4_hdr.time_to_live()
			//	<< ", time=" << (now - time_sent).total_milliseconds() << " ms"
			//	<< std::endl;
			// CONST INT MEASUERETIMES
			// std::cout << "[REPLY NUM] :: => " << num_replies << std::endl;
			if (num_replies < MEASURETIMES) {
				latency += (now - time_sent).total_milliseconds();
				start_send(deltaT);
			}
			else {
				//std::cout << "[CLIENT :: PING] ==> propagation latency (ms): " << latency / MEASURETIMES << std::endl;
				deltaT = latency / MEASURETIMES;
			}
		}
	}

	static unsigned short get_identifier()
	{
#if defined(BOOST_ASIO_WINDOWS)
		return static_cast<unsigned short>(::GetCurrentProcessId());
#else
		return static_cast<unsigned short>(::getpid());
#endif
	}
};
#endif // __ICMP_PING_H