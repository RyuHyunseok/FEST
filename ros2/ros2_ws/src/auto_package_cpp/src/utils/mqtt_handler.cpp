#include "auto_package_cpp/mqtt_handler.hpp"
#include <iostream>

MqttHandler::MqttHandler(const std::string& broker_address, int port)
    : broker_address_(broker_address), port_(port), connected_(false), mosq_(nullptr) {
    int rc = mosquitto_lib_init();
    if (rc != MOSQ_ERR_SUCCESS) {
        std::cerr << "Failed to initialize MQTT library" << std::endl;
        return;
    }

    mosq_ = mosquitto_new(NULL, true, this);
    if (!mosq_) {
        std::cerr << "Failed to create MQTT client" << std::endl;
        mosquitto_lib_cleanup();
        return;
    }

    mosquitto_connect_callback_set(mosq_, connect_callback);
    mosquitto_message_callback_set(mosq_, message_callback);
}

MqttHandler::~MqttHandler() {
    if (mosq_) {
        mosquitto_loop_stop(mosq_, true);
        mosquitto_disconnect(mosq_);
        mosquitto_destroy(mosq_);
        mosq_ = nullptr;
    }
    mosquitto_lib_cleanup();
}

bool MqttHandler::connect() {
    if (!mosq_) return false;

    int rc = mosquitto_connect(mosq_, broker_address_.c_str(), port_, 60);
    if (rc != MOSQ_ERR_SUCCESS) {
        std::cerr << "Failed to connect to MQTT broker" << std::endl;
        return false;
    }

    rc = mosquitto_loop_start(mosq_);
    if (rc != MOSQ_ERR_SUCCESS) {
        std::cerr << "Failed to start MQTT loop" << std::endl;
        mosquitto_disconnect(mosq_);
        return false;
    }

    connected_ = true;
    return true;
}

void MqttHandler::disconnect() {
    if (mosq_) {
        mosquitto_loop_stop(mosq_, true);
        mosquitto_disconnect(mosq_);
        connected_ = false;
    }
}

bool MqttHandler::is_connected() const {
    return connected_;
}

void MqttHandler::set_goal_callback(GoalCallback callback) {
    goal_callback_ = callback;
}

void MqttHandler::connect_callback(struct mosquitto *mosq, void *obj, int result) {
    if (!result) {
        mosquitto_subscribe(mosq, NULL, "incidents/new", 0);
    }
}

void MqttHandler::message_callback(struct mosquitto *mosq, void *obj, const struct mosquitto_message *message) {
    if (!message || !message->payload) {
        return;
    }

    MqttHandler* handler = static_cast<MqttHandler*>(obj);
    if (!handler || !handler->goal_callback_) {
        return;
    }

    try {
        if (std::string(message->topic) == "incidents/new") {
            auto json_data = nlohmann::json::parse(static_cast<char*>(message->payload));
            
            if (!json_data.contains("location") || 
                !json_data["location"].contains("x") || 
                !json_data["location"].contains("y")) {
                std::cerr << "Invalid message format" << std::endl;
                return;
            }

            double x = json_data["location"]["x"].get<double>();
            double y = json_data["location"]["y"].get<double>();

            handler->goal_callback_(x, y);
        }
    }
    catch (const std::exception& e) {
        std::cerr << "Error processing MQTT message: " << e.what() << std::endl;
    }
} 