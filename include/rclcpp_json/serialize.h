#ifndef RCLCPP_JSON_SERIALIZE_H
#define RCLCPP_JSON_SERIALIZE_H

#include <cxxabi.h>
#include <nlohmann/json.hpp>
#include <rclcpp/rclcpp.hpp>
#include <regex>
#include <rosidl_typesupport_cpp/message_type_support.hpp>
#include <rosidl_typesupport_introspection_cpp/field_types.hpp>
#include <rosidl_typesupport_introspection_cpp/message_introspection.hpp>

#include "rosbag2_cpp/typesupport_helpers.hpp"

namespace rclcpp_json {

// 解构函数
inline std::string demangle(const char *name)
{
    int status = 0;
    std::shared_ptr<char> res{abi::__cxa_demangle(name, nullptr, nullptr, &status), std::free};
    return (status == 0) ? res.get() : name;
}

// 转换格式为 package/msg/MessageName
inline std::string format_message_type(const std::string &demangled_name)
{
    std::string formatted_name = demangled_name;

    std::string msg_namespace = "::msg::";
    size_t pos = formatted_name.find(msg_namespace);
    if (pos != std::string::npos)
    {
        formatted_name.replace(pos, msg_namespace.length(), "/msg/");
    }

    while ((pos = formatted_name.find("::")) != std::string::npos)
    {
        formatted_name.replace(pos, 2, "/");
    }

    pos = formatted_name.find("_<");
    if (pos != std::string::npos)
    {
        formatted_name = formatted_name.substr(0, pos);
    }

    return formatted_name;
}

// 获取格式化类型名
template<typename T>
inline std::string get_formatted_type_name(const T &msg)
{
    return format_message_type(demangle(typeid(msg).name()));
}

template<typename T>
inline void serialize_field(nlohmann::json &json_obj, const std::string &field_name, const T &field_value)
{
    json_obj[field_name] = field_value;
}

// 序列化消息字段的主要函数
inline void serialize_message_fields(
    nlohmann::json &json_obj, const void *msg, const rosidl_typesupport_introspection_cpp::MessageMembers *members)
{
    if (!members)
    {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp_json"), "Invalid message members pointer.");
        return;
    }

    RCLCPP_DEBUG(rclcpp::get_logger("rclcpp_json"), "Processing message with %d fields", members->member_count_);
    for (uint32_t i = 0; i < members->member_count_; ++i)
    {
        const rosidl_typesupport_introspection_cpp::MessageMember *member = members->members_ + i;

        if (!member || !member->name_)
        {
            RCLCPP_ERROR(rclcpp::get_logger("rclcpp_json"), "Invalid member or member name in message introspection.");
            continue;
        }

        const void *field_ptr = reinterpret_cast<const char *>(msg) + member->offset_;
        std::string field_name = member->name_;

        if (field_name.empty() ||
            field_name.find_first_not_of("abcdefghijklmnopqrstuvwxyzABCDEFGHIJKLMNOPQRSTUVWXYZ0123456789_") !=
                std::string::npos)
        {
            RCLCPP_ERROR(rclcpp::get_logger("rclcpp_json"), "Invalid field name detected: %s", field_name.c_str());
            continue;
        }

        RCLCPP_DEBUG(rclcpp::get_logger("rclcpp_json"),
            "Serializing field: %s with type_id: %d",
            field_name.c_str(),
            member->type_id_);

        try
        {
            if (member->is_array_)
            {
                nlohmann::json array_json = nlohmann::json::array();

                // 获取数组大小
                size_t array_size = member->array_size_;
                if (member->size_function)
                {
                    array_size = member->size_function(field_ptr);
                }

                for (size_t j = 0; j < array_size; ++j)
                {
                    const void *element_ptr = nullptr;
                    if (member->get_const_function)
                    {
                        element_ptr = member->get_const_function(field_ptr, j);
                    }
                    else
                    {
                        element_ptr = reinterpret_cast<const char *>(field_ptr) + j * member->size_function(field_ptr);
                    }

                    switch (member->type_id_)
                    {
                    case rosidl_typesupport_introspection_cpp::ROS_TYPE_BOOL:
                        array_json.push_back(*reinterpret_cast<const bool *>(element_ptr));
                        break;
                    case rosidl_typesupport_introspection_cpp::ROS_TYPE_BYTE:
                    case rosidl_typesupport_introspection_cpp::ROS_TYPE_UINT8:
                        array_json.push_back(*reinterpret_cast<const uint8_t *>(element_ptr));
                        break;
                    case rosidl_typesupport_introspection_cpp::ROS_TYPE_CHAR:
                    case rosidl_typesupport_introspection_cpp::ROS_TYPE_INT8:
                        array_json.push_back(*reinterpret_cast<const int8_t *>(element_ptr));
                        break;
                    case rosidl_typesupport_introspection_cpp::ROS_TYPE_UINT16:
                        array_json.push_back(*reinterpret_cast<const uint16_t *>(element_ptr));
                        break;
                    case rosidl_typesupport_introspection_cpp::ROS_TYPE_INT16:
                        array_json.push_back(*reinterpret_cast<const int16_t *>(element_ptr));
                        break;
                    case rosidl_typesupport_introspection_cpp::ROS_TYPE_UINT32:
                        array_json.push_back(*reinterpret_cast<const uint32_t *>(element_ptr));
                        break;
                    case rosidl_typesupport_introspection_cpp::ROS_TYPE_INT32:
                        array_json.push_back(*reinterpret_cast<const int32_t *>(element_ptr));
                        break;
                    case rosidl_typesupport_introspection_cpp::ROS_TYPE_UINT64:
                        array_json.push_back(*reinterpret_cast<const uint64_t *>(element_ptr));
                        break;
                    case rosidl_typesupport_introspection_cpp::ROS_TYPE_INT64:
                        array_json.push_back(*reinterpret_cast<const int64_t *>(element_ptr));
                        break;
                    case rosidl_typesupport_introspection_cpp::ROS_TYPE_FLOAT:
                        array_json.push_back(*reinterpret_cast<const float *>(element_ptr));
                        break;
                    case rosidl_typesupport_introspection_cpp::ROS_TYPE_DOUBLE:
                        array_json.push_back(*reinterpret_cast<const double *>(element_ptr));
                        break;
                    case rosidl_typesupport_introspection_cpp::ROS_TYPE_STRING:
                        array_json.push_back(*reinterpret_cast<const std::string *>(element_ptr));
                        break;
                    case rosidl_typesupport_introspection_cpp::ROS_TYPE_MESSAGE: {
                        const rosidl_typesupport_introspection_cpp::MessageMembers *nested_members =
                            static_cast<const rosidl_typesupport_introspection_cpp::MessageMembers *>(
                                member->members_->data);
                        nlohmann::json nested_json;
                        serialize_message_fields(nested_json, element_ptr, nested_members);
                        array_json.push_back(nested_json);
                    }
                    break;
                    default:
                        RCLCPP_ERROR(rclcpp::get_logger("rclcpp_json"),
                            "Unsupported array element type for field: %s with type id: %d",
                            field_name.c_str(),
                            member->type_id_);
                        break;
                    }
                }

                json_obj[field_name] = array_json;
            }
            else
            {
                switch (member->type_id_)
                {
                case rosidl_typesupport_introspection_cpp::ROS_TYPE_BOOL:
                    serialize_field(json_obj, field_name, *reinterpret_cast<const bool *>(field_ptr));
                    break;
                case rosidl_typesupport_introspection_cpp::ROS_TYPE_BYTE:
                case rosidl_typesupport_introspection_cpp::ROS_TYPE_UINT8:
                    serialize_field(json_obj, field_name, *reinterpret_cast<const uint8_t *>(field_ptr));
                    break;
                case rosidl_typesupport_introspection_cpp::ROS_TYPE_CHAR:
                case rosidl_typesupport_introspection_cpp::ROS_TYPE_INT8:
                    serialize_field(json_obj, field_name, *reinterpret_cast<const int8_t *>(field_ptr));
                    break;
                case rosidl_typesupport_introspection_cpp::ROS_TYPE_UINT16:
                    serialize_field(json_obj, field_name, *reinterpret_cast<const uint16_t *>(field_ptr));
                    break;
                case rosidl_typesupport_introspection_cpp::ROS_TYPE_INT16:
                    serialize_field(json_obj, field_name, *reinterpret_cast<const int16_t *>(field_ptr));
                    break;
                case rosidl_typesupport_introspection_cpp::ROS_TYPE_UINT32:
                    serialize_field(json_obj, field_name, *reinterpret_cast<const uint32_t *>(field_ptr));
                    break;
                case rosidl_typesupport_introspection_cpp::ROS_TYPE_INT32:
                    serialize_field(json_obj, field_name, *reinterpret_cast<const int32_t *>(field_ptr));
                    break;
                case rosidl_typesupport_introspection_cpp::ROS_TYPE_UINT64:
                    serialize_field(json_obj, field_name, *reinterpret_cast<const uint64_t *>(field_ptr));
                    break;
                case rosidl_typesupport_introspection_cpp::ROS_TYPE_INT64:
                    serialize_field(json_obj, field_name, *reinterpret_cast<const int64_t *>(field_ptr));
                    break;
                case rosidl_typesupport_introspection_cpp::ROS_TYPE_FLOAT:
                    serialize_field(json_obj, field_name, *reinterpret_cast<const float *>(field_ptr));
                    break;
                case rosidl_typesupport_introspection_cpp::ROS_TYPE_DOUBLE:
                    serialize_field(json_obj, field_name, *reinterpret_cast<const double *>(field_ptr));
                    break;
                case rosidl_typesupport_introspection_cpp::ROS_TYPE_STRING:
                    serialize_field(json_obj, field_name, *reinterpret_cast<const std::string *>(field_ptr));
                    break;
                case rosidl_typesupport_introspection_cpp::ROS_TYPE_MESSAGE: {
                    const rosidl_typesupport_introspection_cpp::MessageMembers *nested_members =
                        static_cast<const rosidl_typesupport_introspection_cpp::MessageMembers *>(
                            member->members_->data);
                    nlohmann::json nested_json;
                    serialize_message_fields(nested_json, field_ptr, nested_members);
                    json_obj[field_name] = nested_json;
                }
                break;
                default:
                    RCLCPP_ERROR(rclcpp::get_logger("rclcpp_json"),
                        "Unsupported field type for field: %s with type id: %d",
                        field_name.c_str(),
                        member->type_id_);
                    break;
                }
            }
        }
        catch (const std::exception &e)
        {
            RCLCPP_ERROR(rclcpp::get_logger("rclcpp_json"),
                "Exception during serialization of field %s: %s",
                field_name.c_str(),
                e.what());
        }
    }
}

// 将任意ROS2消息转换为JSON
template<typename T>
inline nlohmann::json serialize_to_json(const T &msg)
{
    std::string type_name = get_formatted_type_name(msg);

    nlohmann::json json_obj;

    auto introspection_library =
        rosbag2_cpp::get_typesupport_library(type_name, "rosidl_typesupport_introspection_cpp");
    if (!introspection_library)
    {
        std::cerr << "Error: Could not get typesupport library for type: " << type_name << std::endl;
        return json_obj;
    }

    auto introspection_support =
        rosbag2_cpp::get_typesupport_handle(type_name, "rosidl_typesupport_introspection_cpp", introspection_library);
    if (!introspection_support)
    {
        std::cerr << "Error: Could not get typesupport handle for type: " << type_name << std::endl;
        return json_obj;
    }

    const auto *members =
        static_cast<const rosidl_typesupport_introspection_cpp::MessageMembers *>(introspection_support->data);

    if (!members)
    {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp_json"), "Failed to get message type support handle.");
        return json_obj;
    }

    serialize_message_fields(json_obj, &msg, members);

    return json_obj;
}

} // namespace rclcpp_json

#endif // RCLCPP_JSON_SERIALIZE_H
