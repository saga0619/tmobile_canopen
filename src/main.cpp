#include <canopen_master/canopen.h>
#include <boost/make_shared.hpp>
#include <iostream>

#include <socketcan_interface/dispatcher.h>
#include <boost/unordered_set.hpp>
#include <socketcan_interface/socketcan.h>
#include <socketcan_interface/threading.h>
#include <socketcan_interface/make_shared.h>

#include <boost/thread.hpp>

using namespace can;
using namespace canopen;

class Cobra4812Node : public Node
{
    static ObjectDictSharedPtr make_dict()
    {
        canopen::DeviceInfo info;
        info.nr_of_rx_pdo = 0;
        info.nr_of_tx_pdo = 0;

        ObjectDictSharedPtr dict = ROSCANOPEN_MAKE_SHARED<canopen::ObjectDict>(info);

        // dict->insert(true, ROSCANOPEN_MAKE_SHARED<const canopen::ObjectDict::Entry>(0x1023, 1, ObjectDict::DEFTYPE_OCTET_STRING, "Command", false, true, false));
        // dict->insert(true, ROSCANOPEN_MAKE_SHARED<const canopen::ObjectDict::Entry>(0x1023, 2, ObjectDict::DEFTYPE_UNSIGNED8, "Status", true, false, false));
        // dict->insert(true, ROSCANOPEN_MAKE_SHARED<const canopen::ObjectDict::Entry>(0x1023, 3, ObjectDict::DEFTYPE_OCTET_STRING, "Reply", true, false, false));

        dict->insert(false, ROSCANOPEN_MAKE_SHARED<const canopen::ObjectDict::Entry>(ObjectDict::VAR, 0x6040, ObjectDict::DEFTYPE_UNSIGNED16, "Control word", true, true, true));

        dict->insert(false, ROSCANOPEN_MAKE_SHARED<const canopen::ObjectDict::Entry>(ObjectDict::VAR, 0x6041, ObjectDict::DEFTYPE_UNSIGNED16, "Status value", true, false, true));

        dict->insert(false, ROSCANOPEN_MAKE_SHARED<const canopen::ObjectDict::Entry>(ObjectDict::VAR, 0x1017, ObjectDict::DEFTYPE_UNSIGNED16, "Heartbeat message time", true, true, false, HoldAny((uint16_t)0)));

        dict->insert(false, ROSCANOPEN_MAKE_SHARED<const canopen::ObjectDict::Entry>(ObjectDict::VAR, 0x6063, ObjectDict::DEFTYPE_INTEGER32, "Actual position value", true, false, true));


        dict->insert(false, ROSCANOPEN_MAKE_SHARED<const canopen::ObjectDict::Entry>(ObjectDict::VAR, 0x200B, ObjectDict::DEFTYPE_UNSIGNED32, "error code", true, false, false));


        // dict->insert(false, )

        return dict;
    }

    ObjectStorage::Entry<canopen::ObjectStorage::DataType<ObjectDict::DEFTYPE_UNSIGNED16>::type> control_word_;
    ObjectStorage::Entry<canopen::ObjectStorage::DataType<ObjectDict::DEFTYPE_UNSIGNED16>::type> status_value_;
    ObjectStorage::Entry<canopen::ObjectStorage::DataType<ObjectDict::DEFTYPE_UNSIGNED16>::type> heartbeat_time_;
    ObjectStorage::Entry<canopen::ObjectStorage::DataType<ObjectDict::DEFTYPE_INTEGER32>::type> actual_position_;
    ObjectStorage::Entry<canopen::ObjectStorage::DataType<ObjectDict::DEFTYPE_UNSIGNED32>::type> error_code_;



    template <typename T>
    bool try_set(T &entry, const typename T::type &value)
    {
        try
        {
            entry.set(value);
        }
        catch (...)
        {
            return false;
        }
        return true;
    }

public:
    Cobra4812Node(can::CommInterfaceSharedPtr interface, const uint8_t &id) : Node(interface, make_dict(), id)
    {
        // getStorage()->entry(command_, 0x1023, 1);
        // getStorage()->entry(status_, 0x1023, 2);
        // getStorage()->entry(reply_, 0x1023, 3);

        // getStorage()->entry(mode_, 0x1024);

        getStorage()->entry(control_word_, 0x6040);
        getStorage()->entry(status_value_, 0x6041);
        getStorage()->entry(heartbeat_time_, 0x1017);
        getStorage()->entry(actual_position_, 0x6063);
        getStorage()->entry(error_code_, 0x200B);
    }

    uint16_t get_status()
    {
        uint16_t res;
        try
        {
            res = status_value_.get();
        }
        catch (const TimeoutException &)
        {
            std::cout << "TIMEOUT" << std::endl;
            res = 0;
        }
        return res;
    }

    uint32_t get_errorcode()
    {
        uint32_t res;
        try
        {
            res = error_code_.get();
        }
        catch (const TimeoutException &)
        {
            std::cout << "TIMEOUT" << std::endl;
            res = 0;
        }
        return res;
    }

    int32_t get_position()
    {
        int32_t pos;
        try
        {
            pos = actual_position_.get();
        }
        catch (const TimeoutException &)
        {
            std::cout << "TIMEOUT" << std::endl;
            pos = 0;
        }
        return pos;
    }

    int set_controlword(uint16_t word)
    {
        if(try_set(control_word_,word))
        {
            std::cout<<"control word set complete"<<std::endl;
            return 1;
        }
        else{
            std::cout<<"control word set failed"<<std::endl;
            return 0;
        }
    }


    // uint8_t send_command(const std::string &s){
    //     if(!try_set(command_, String(s))) return 0;

    //     uint8_t status;

    //     do{
    //         status = status_.get();
    //     }while(status == 255);

    //     return status;
    // }
    // void get_response(std::string &s){
    //     String res;
    //     try{
    //         res = reply_.get();
    //         s = (const std::string&) res;
    //     }
    //     catch(const TimeoutException&){
    //         s = "<TIMEOUT>";
    //     }
    // }
};

ThreadedSocketCANInterfaceSharedPtr driver;
boost::shared_ptr<Cobra4812Node> node;
StateListenerConstSharedPtr state_printer;

void print_state(const State &s)
{
    ThreadedSocketCANInterfaceSharedPtr d = driver;
    std::string msg;
    if (!s.error_code && !s.internal_error)
        return;

    if (d && !d->translateError(s.internal_error, msg))
        msg = "Undefined";
    std::cerr << "device error: " << s.error_code << " internal_error: " << s.internal_error << " (" << msg << ")" << std::endl;
}

void shutdown(ThreadedSocketCANInterfaceSharedPtr &driver, boost::shared_ptr<Cobra4812Node> &node, int code)
{
    state_printer.reset();

    LayerStatus s;
    if (node)
        node->shutdown(s);
    if (driver)
        driver->shutdown();
    exit(code);
}

void sigint_handler(int param)
{
    shutdown(driver, node, param);
}

int main(int argc, char *argv[])
{

    if (argc <= 2)
    {
        std::cerr << "usage: " << argv[0] << " device node_id [< INPUT] [> OUTPUT]" << std::endl;
        return 1;
    }

    bool tty = isatty(fileno(stdin));

    uint8_t nodeid = atoi(argv[2]);

    signal(SIGINT, sigint_handler);

    driver = ROSCANOPEN_MAKE_SHARED<ThreadedSocketCANInterface>();
    state_printer = driver->createStateListener(print_state);

    if (!driver->init(argv[1], 0))
    {
        std::cerr << "init failed" << std::endl;
        return 1;
    }

    sleep(1.0);

    node = boost::make_shared<Cobra4812Node>(driver, nodeid);

    LayerStatus status;
    try
    {
        node->init(status);
        if (!status.bounded<LayerStatus::Warn>())
        {
            std::cerr << status.reason() << std::endl;
            shutdown(driver, node, 1);
        }
    }
    catch (const canopen::Exception &e)
    {
        std::cerr << boost::diagnostic_information(e) << std::endl;
        shutdown(driver, node, 1);
    }

    std::string res;
    std::string line;

    std::cout << "Starting ... " << std::endl;

    uint16_t r1 = node->get_status();
    int32_t r2 = node->get_position();


    std::cout<<"status : "<<r1<<"  pos : "<<r2<<" error : " << node->get_errorcode()<<std::endl;



    
    // while(std::cin.good()){
    //     if(tty) std::cout << "> ";
    //     std::getline (std::cin, line);

    //     if(line.empty()) continue;

    //     if(!tty) std::cout << line <<  ": ";

    //     uint8_t status = node->send_command(line);
    //     if(status){
    //         node->get_response(res);
    //         std::cout << (status == 1 ? res : "<ERROR>")  << std::endl;
    //     }else{
    //         std::cout << "ERROR!";
    //         break;
    //     }
    // }

    shutdown(driver, node, 0);

    return 0;
}
