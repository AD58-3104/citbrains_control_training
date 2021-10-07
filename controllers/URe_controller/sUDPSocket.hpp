#ifndef sUDPSOCKET_H
#define sUDPSOCKET_H

#include <iostream>
#include <boost/asio.hpp>
#include <boost/thread.hpp>
#include <boost/array.hpp>
#include <thread>
#include <memory>
#include <chrono>
#include <string>
#include <atomic>
using boost::asio::ip::udp;
using namespace std::literals::chrono_literals;

#if (((BOOST_VERSION / 100000) + ((BOOST_VERSION) / 100 % 1000)) > 165)
//Boost1.65より新しい時(io_serviceがdeprecatedな為分けている)
/**/ #define BOOST_VERSION_IS_HIGHER_THAN_1_65
#else
// Boost1.65以下の時
/**/ #define BOOST_VERSION_IS_1_65_OR_LOWER
#endif //(((BOOST_VERSION / 1000) + ((BOOST_VERSION) % 1000)) >= 165)

    namespace Citbrains
{
    namespace Udpsocket
    {
        namespace SocketMode
        {
            using udpsocketmode_t = int32_t;
            static inline constexpr udpsocketmode_t broadcast_mode = 0;
            static inline constexpr udpsocketmode_t multicast_mode = broadcast_mode + 1;
            static inline constexpr udpsocketmode_t unicast_mode = multicast_mode + 1;
        }
        class UDPClient
        {
            boost::asio::io_service io_service_;
            udp::socket socket_;
            int cnt;
            int32_t port_;
            std::string ip_address_;
            bool allow_broadcast_;
            bool allow_multicast_;
            std::unique_ptr<std::thread> client_thread_;
            bool terminated_;
#ifdef BOOST_VERSION_IS_HIGHER_THAN_1_65
            boost::asio::executor_work_guard<boost::asio::io_context::executor_type> w_guard_;
#else
            std::shared_ptr<boost::asio::io_service::work> w_guard_;
#endif //BOOST_VERSION_IS_HIGHER_THAN_1_65

        public:
            /**
             * @fn
             * コンストラクタ
             * @brief コンストラクタ
             * @param (address) 送り先のIPアドレス.
             * @param (port) 送り先のポート番号
             * @detail 引数はマルチキャストの場合マルチキャスト用のを入れる.
             */
            UDPClient(std::string address, int32_t port, SocketMode::udpsocketmode_t mode)
                : io_service_(), socket_(io_service_), cnt(0), port_(port), ip_address_(address), allow_broadcast_(false), terminated_(false),
#ifdef BOOST_VERSION_IS_HIGHER_THAN_1_65
                  w_guard_(boost::asio::make_work_guard(io_service_))
#else
                  w_guard_(std::make_shared<boost::asio::io_service::work>(io_service_))
#endif //BOOST_VERSION_IS_HIGHER_THAN_1_65
            {
                assert((SocketMode::broadcast_mode <= mode) && (mode <= SocketMode::unicast_mode));
                try
                {
                    if (socket_.is_open())
                    {
                        socket_.close();
                    }
                    socket_.open(udp::v4());
                    socket_.set_option(boost::asio::ip::udp::socket::reuse_address(true));
                    if (mode == SocketMode::broadcast_mode)
                    {
                        socket_.set_option(boost::asio::socket_base::broadcast(true));
                    }
                    else if (mode == SocketMode::multicast_mode)
                    {
                        socket_.set_option(boost::asio::ip::multicast::enable_loopback(true));
                        socket_.set_option(boost::asio::ip::multicast::join_group(boost::asio::ip::address::from_string(ip_address_)));
                        allow_multicast_ = true;
                    }
                    else
                    {
                        //TODO:何か書く必要あるっけここ？
                    }
                    boost::asio::socket_base::broadcast broadcast_option;
                    socket_.get_option(broadcast_option);
                    allow_broadcast_ = broadcast_option.value();
                }
                catch (boost::system::system_error &e)
                {
                    std::cerr << "error thrown in " << __FILE__ << "::" << __LINE__ << std::endl;
                    std::cerr << e.what() << std::endl;
                }

                client_thread_ = std::make_unique<std::thread>([&]()
                                                               { io_service_.run(); });
            }
            ~UDPClient()
            {
                terminate();
            }
            UDPClient(const UDPClient &) = delete;
            UDPClient &operator=(const UDPClient &) = delete;

            // メッセージ送信
            void send(std::string &&bytestring)
            {
                std::string send_data = std::move(bytestring);
                if (allow_broadcast_)
                {
                    boost::asio::ip::udp::endpoint destination(boost::asio::ip::address_v4::broadcast(), port_);
                    socket_.async_send_to(
                        boost::asio::buffer(send_data),
                        destination,
                        [this](const boost::system::error_code &error, size_t bytes_transferred)
                        { sendHandler(error, bytes_transferred); });
                }
                else
                {
                    boost::asio::ip::udp::endpoint destination(boost::asio::ip::address::from_string(ip_address_), port_);
                    socket_.async_send_to(
                        boost::asio::buffer(send_data),
                        destination,
                        [this](const boost::system::error_code &error, size_t bytes_transferred)
                        { sendHandler(error, bytes_transferred); });
                }
            }

            // 送信のハンドラ
            void sendHandler(const boost::system::error_code &error, size_t bytes_transferred)
            {
#ifdef DEBUG_SUDPSOCKET
                if (error)
                {
                    std::cout << "send failed: " << error.message() << std::endl; //TODO::どこかに書き込む
                }
                else
                {
                    std::cout << "send correct!" << std::endl;
                }
                static int32_t send_cnt = 0;
                send_cnt++;
                std::cout << "send message " << send_cnt << "times" << std::endl;
#endif //DEBUG_SUDPSOCKET
            }

            void terminate()
            {
                // static bool already_called = false;
                if (!terminated_)
                {
                    try
                    {
                        terminated_ = true;
                        w_guard_.reset(); //ここでwork_guardかworkを破棄してrun()のブロッキングを終わらせる
                        io_service_.stop();
                        socket_.close();
                        client_thread_->join();
                        std::cout << "client is terminated!!" << std::endl;
                    }
                    catch (const std::runtime_error &e)
                    {
                        std::cout << "exception catched in" << __FILE__ << __LINE__ << std::endl;
                        std::cout << e.what() << std::endl;
                    }
                }
            }
        };

        class UDPServer
        {

            boost::asio::io_service io_service_;
            udp::socket socket_;
            udp::endpoint remote_endpoint_;
            bool terminated_;
            boost::asio::streambuf receive_buff_;
            inline static constexpr int32_t buffer_size_ = 1024;
            // boost::array<char, buffer_size_> receive_buff_;
            int32_t port_;
            std::function<void(std::string &&s)> receivedHandler_;
            std::unique_ptr<std::thread> server_thread_;
#ifdef BOOST_VERSION_IS_HIGHER_THAN_1_65
            boost::asio::executor_work_guard<boost::asio::io_context::executor_type> w_guard_;
#else
            std::shared_ptr<boost::asio::io_service::work> w_guard_;
#endif //BOOST_VERSION_IS_HIGHER_THAN_1_65

        public:
            /**
            * @fn
            * コンストラクタ
            * @brief コンストラクタ
            * @param (port) サーバーがデータを受け取るためのポート番号
            * @param (func) 受け取ったデータ(文字列)を処理する為の関数オブジェクト
            * @detail 第二引数の関数オブジェクトはstd::stringの右辺値参照を取る。呼び出した時点から受信待機を行う。
            */
            UDPServer(int32_t port, std::function<void(std::string &&)> func, SocketMode::udpsocketmode_t mode, int32_t handler_work_pararell_num = 1, std::string multicast_address = "224.0.0.169")
                : io_service_(),
                  socket_(io_service_, udp::endpoint(udp::v4(), port)), terminated_(false), port_(port), receivedHandler_(func),
#ifdef BOOST_VERSION_IS_HIGHER_THAN_1_65
                  w_guard_(boost::asio::make_work_guard(io_service_))
#else
                  w_guard_(std::make_shared<boost::asio::io_service::work>(io_service_))
#endif //BOOST_VERSION_IS_HIGHER_THAN_1_65
            {
                try
                {
                    socket_.set_option(boost::asio::ip::udp::socket::reuse_address(true));
                    if (mode == SocketMode::multicast_mode)
                    {
                        socket_.set_option(boost::asio::ip::multicast::enable_loopback(true));
                        socket_.set_option(boost::asio::ip::multicast::join_group(boost::asio::ip::address::from_string(multicast_address)));
                    }
                    else
                    {
                        //TODO:broadcastとunicastは何もない。
                    }
                    server_thread_ = std::make_unique<std::thread>([&]()
                                                                   {
#ifdef DEBUG_SUDPSOCKET
                                                                       try
#endif // DEBUG_SUDPSOCKET
                                                                       {
                                                                           io_service_.run();
                                                                       }
#ifdef DEBUG_SUDPSOCKET
                                                                       catch (const std::runtime_error &e)
                                                                       {
                                                                        std::cout << "exception catched in sUDPSocket server thread" << std::endl;
                                                                           std::cout << e.what() << std::endl;
                                                                       }
                                                                       std::cout << "server io_service is finished!!!" << std::endl;
#endif // DEBUG_SUDPSOCKET
                                                                   });
                }
                catch (const std::runtime_error &e)
                {
                    std::cout << "exception catched in" << __FILE__ << "::" << __LINE__ << std::endl;
                    std::cout << e.what() << std::endl;
                }
                startReceive();
            }
            ~UDPServer()
            {
                terminate();
            }
            UDPServer(const UDPServer &) = delete;
            UDPServer &operator=(const UDPServer &) = delete;
            // 受信開始。コンストラクタで呼ばれる。
            void startReceive()
            {
#ifdef DEBUG_SUDPSOCKET
                try
#endif // DEBUG_SUDPSOCKET
                {
                    if (!terminated_)
                        socket_.async_receive_from(
                            receive_buff_.prepare(buffer_size_),
                            // boost::asio::buffer(receive_buff_),
                            remote_endpoint_,
                            // boost::bind(&UDPServer::receiveHandler, this,
                            //             asio::placeholders::error, asio::placeholders::bytes_transferred)
                            [this](const boost::system::error_code &error, size_t bytes_transferred)
                            {
                                receiveHandler(error, bytes_transferred);
                            });
                }
#ifdef DEBUG_SUDPSOCKET
                catch (const std::runtime_error &e)
                {
                    std::cout << "exception catched in reiceve" << std::endl;
                    std::cout << e.what() << std::endl;
                }
#endif // DEBUG_SUDPSOCKET
            }

            // 受信のハンドラ。データを受け取った時に呼ばれる。
            void receiveHandler(const boost::system::error_code &error, size_t bytes_transferred)
            {
                if (error && error != boost::asio::error::eof)
                {
#ifdef DEBUG_SUDPSOCKET
                    std::cout << "receive failed: " << error.message() << std::endl;
#endif
                }
                else
                {
                    // std::string data(receive_buff_.data(), bytes_transferred);
                    std::string data(boost::asio::buffer_cast<const char *>(receive_buff_.data()), bytes_transferred);
#ifdef DEBUG_SUDPSOCKET
                    std::cout << "length::" << bytes_transferred << " row data is ==" << data << std::endl;
#endif
                    receivedHandler_(std::move(data));
                    receive_buff_.consume(receive_buff_.size());
                    if (!terminated_)
                    {
                        startReceive();
                    }
                }
            }
            /**
             * @fn
             * 終了関数
             * @brief サーバーを停止する時に呼び出す。デストラクタでも呼び出されるので、インスタンスの寿命を管理するならわざわざ呼び出さなくても終了可能。 
             */
            void terminate()
            {
                if (!terminated_)
                {
                    try
                    {
                        terminated_ = true;
                        w_guard_.reset();
                        // socket_.cancel();
                        socket_.close();
                        io_service_.stop();
                        server_thread_->join();
                        std::cout << "server is terminated!!" << std::endl;
                    }
                    catch (const std::runtime_error &e)
                    {
                        std::cout << "exception catched in" << __FILE__ << __LINE__ << std::endl;
                        std::cout << e.what() << std::endl;
                    }
                }
            };
        };
    }
}

#endif 