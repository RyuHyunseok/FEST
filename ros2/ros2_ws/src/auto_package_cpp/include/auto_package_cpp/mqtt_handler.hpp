#ifndef MQTT_HANDLER_HPP
#define MQTT_HANDLER_HPP

#include <mosquitto.h>
#include <nlohmann/json.hpp>
#include <functional>
#include <string>

class MqttHandler {
public:
    MqttHandler(const std::string& broker_address = "localhost", int port = 1883);
    ~MqttHandler();

    bool connect();
    void disconnect();
    bool is_connected() const;
    
    // 콜백 함수 타입 정의
    using GoalCallback = std::function<void(double x, double y)>;
    
    // 콜백 함수 설정
    void set_goal_callback(GoalCallback callback);

private:
    static void connect_callback(struct mosquitto *mosq, void *obj, int result);
    static void message_callback(struct mosquitto *mosq, void *obj, const struct mosquitto_message *message);

    struct mosquitto *mosq_;
    std::string broker_address_;
    int port_;
    bool connected_;
    GoalCallback goal_callback_;
};

#endif // MQTT_HANDLER_HPP 