#ifndef RCLCPP_JSON_DESERIALIZE_H
#define RCLCPP_JSON_DESERIALIZE_H

#include <nlohmann/json.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rosidl_typesupport_cpp/message_type_support.hpp>
#include <rosidl_typesupport_introspection_cpp/field_types.hpp>
#include <rosidl_typesupport_introspection_cpp/message_introspection.hpp>

namespace rclcpp_json {

// 辅助函数：从 JSON 对象中反序列化字段
template<typename T>
void deserialize_field(const nlohmann::json &json_obj, const std::string &field_name, T &field_value)
{
    field_value = json_obj.at(field_name).get<T>();
}

// 递归处理嵌套消息
void deserialize_nested_message(void *field_ptr,
    const nlohmann::json &json_obj,
    const rosidl_typesupport_introspection_cpp::MessageMembers *members);

// 反序列化消息字段的主要函数
void deserialize_message_fields(
    void *msg, const nlohmann::json &json_obj, const rosidl_typesupport_introspection_cpp::MessageMembers *members)
{
    for (uint32_t i = 0; i < members->member_count_; ++i)
    {
        const rosidl_typesupport_introspection_cpp::MessageMember *member = &members->members_[i];
        void *field_ptr = static_cast<char *>(msg) + member->offset_;
        std::string field_name = member->name_;

        if (!json_obj.contains(field_name))
        {
            RCLCPP_WARN(rclcpp::get_logger("roscpp_json"), "Field '%s' not found in JSON object", field_name.c_str());
            continue;
        }

        // 根据字段是数组还是单个元素进行反序列化
        if (member->is_array_)
        {
            nlohmann::json array_json = json_obj.at(field_name);
            if (!array_json.is_array())
            {
                throw std::runtime_error("Expected an array for field: " + field_name);
            }

            size_t array_size = array_json.size();
            size_t member_array_size = member->array_size_;
            bool is_bounded = member->is_upper_bound_;

            if (member->resize_function && (!is_bounded || array_size <= member_array_size))
            {
                // 需要时调整数组大小
                member->resize_function(field_ptr, array_size);
            }
            else if (array_size > member_array_size)
            {
                throw std::runtime_error("Array size for field '" + field_name + "' exceeds the maximum size");
            }
            
            for (size_t j = 0; j < array_size; ++j)
            {
                // 确保元素指针计算正确
                const void *element_ptr = nullptr;
                if (member->get_const_function) {
                    element_ptr = member->get_const_function(field_ptr, j);
                } else {
                    // 默认方式：通过偏移量计算元素指针
                    element_ptr = reinterpret_cast<const char *>(field_ptr) + j * member->size_function(field_ptr);
                }

                if (!element_ptr)
                {
                    throw std::runtime_error("Invalid pointer for array element at index " + std::to_string(j));
                }

                // 根据类型反序列化每个元素
                switch (member->type_id_)
                {
                case rosidl_typesupport_introspection_cpp::ROS_TYPE_INT32:
                    *const_cast<int32_t *>(reinterpret_cast<const int32_t *>(element_ptr)) = array_json.at(j).get<int32_t>();
                    break;
                case rosidl_typesupport_introspection_cpp::ROS_TYPE_FLOAT:
                    *const_cast<float *>(reinterpret_cast<const float *>(element_ptr)) = array_json.at(j).get<float>();
                    break;
                case rosidl_typesupport_introspection_cpp::ROS_TYPE_STRING:
                    *const_cast<std::string *>(reinterpret_cast<const std::string *>(element_ptr)) = array_json.at(j).get<std::string>();
                    break;
                case rosidl_typesupport_introspection_cpp::ROS_TYPE_BOOL:
                    *const_cast<bool *>(reinterpret_cast<const bool *>(element_ptr)) = array_json.at(j).get<bool>();
                    break;
                case rosidl_typesupport_introspection_cpp::ROS_TYPE_BYTE:
                case rosidl_typesupport_introspection_cpp::ROS_TYPE_UINT8:
                    *const_cast<uint8_t *>(reinterpret_cast<const uint8_t *>(element_ptr)) = array_json.at(j).get<uint8_t>();
                    break;
                case rosidl_typesupport_introspection_cpp::ROS_TYPE_INT8:
                    *const_cast<int8_t *>(reinterpret_cast<const int8_t *>(element_ptr)) = array_json.at(j).get<int8_t>();
                    break;
                case rosidl_typesupport_introspection_cpp::ROS_TYPE_UINT16:
                    *const_cast<uint16_t *>(reinterpret_cast<const uint16_t *>(element_ptr)) = array_json.at(j).get<uint16_t>();
                    break;
                case rosidl_typesupport_introspection_cpp::ROS_TYPE_INT16:
                    *const_cast<int16_t *>(reinterpret_cast<const int16_t *>(element_ptr)) = array_json.at(j).get<int16_t>();
                    break;
                case rosidl_typesupport_introspection_cpp::ROS_TYPE_UINT32:
                    *const_cast<uint32_t *>(reinterpret_cast<const uint32_t *>(element_ptr)) = array_json.at(j).get<uint32_t>();
                    break;
                case rosidl_typesupport_introspection_cpp::ROS_TYPE_INT64:
                    *const_cast<int64_t *>(reinterpret_cast<const int64_t *>(element_ptr)) = array_json.at(j).get<int64_t>();
                    break;
                case rosidl_typesupport_introspection_cpp::ROS_TYPE_UINT64:
                    *const_cast<uint64_t *>(reinterpret_cast<const uint64_t *>(element_ptr)) = array_json.at(j).get<uint64_t>();
                    break;
                case rosidl_typesupport_introspection_cpp::ROS_TYPE_DOUBLE:
                    *const_cast<double *>(reinterpret_cast<const double *>(element_ptr)) = array_json.at(j).get<double>();
                    break;
                case rosidl_typesupport_introspection_cpp::ROS_TYPE_MESSAGE: {
                    const rosidl_typesupport_introspection_cpp::MessageMembers *nested_members =
                        static_cast<const rosidl_typesupport_introspection_cpp::MessageMembers *>(member->members_->data);
                    deserialize_nested_message(const_cast<void*>(element_ptr), array_json.at(j), nested_members);
                }
                break;
                default:
                    throw std::runtime_error("Unsupported array element type for field: " + field_name);
                }
            }
        }
        else
        {
            // 处理非数组字段
            switch (member->type_id_)
            {
            case rosidl_typesupport_introspection_cpp::ROS_TYPE_INT32:
                deserialize_field(json_obj, field_name, *reinterpret_cast<int32_t *>(field_ptr));
                break;
            case rosidl_typesupport_introspection_cpp::ROS_TYPE_FLOAT:
                deserialize_field(json_obj, field_name, *reinterpret_cast<float *>(field_ptr));
                break;
            case rosidl_typesupport_introspection_cpp::ROS_TYPE_STRING:
                deserialize_field(json_obj, field_name, *reinterpret_cast<std::string *>(field_ptr));
                break;
            case rosidl_typesupport_introspection_cpp::ROS_TYPE_BOOL:
                deserialize_field(json_obj, field_name, *reinterpret_cast<bool *>(field_ptr));
                break;
            case rosidl_typesupport_introspection_cpp::ROS_TYPE_BYTE:
            case rosidl_typesupport_introspection_cpp::ROS_TYPE_UINT8:
                deserialize_field(json_obj, field_name, *reinterpret_cast<uint8_t *>(field_ptr));
                break;
            case rosidl_typesupport_introspection_cpp::ROS_TYPE_INT8:
                deserialize_field(json_obj, field_name, *reinterpret_cast<int8_t *>(field_ptr));
                break;
            case rosidl_typesupport_introspection_cpp::ROS_TYPE_UINT16:
                deserialize_field(json_obj, field_name, *reinterpret_cast<uint16_t *>(field_ptr));
                break;
            case rosidl_typesupport_introspection_cpp::ROS_TYPE_INT16:
                deserialize_field(json_obj, field_name, *reinterpret_cast<int16_t *>(field_ptr));
                break;
            case rosidl_typesupport_introspection_cpp::ROS_TYPE_UINT32:
                deserialize_field(json_obj, field_name, *reinterpret_cast<uint32_t *>(field_ptr));
                break;
            case rosidl_typesupport_introspection_cpp::ROS_TYPE_INT64:
                deserialize_field(json_obj, field_name, *reinterpret_cast<int64_t *>(field_ptr));
                break;
            case rosidl_typesupport_introspection_cpp::ROS_TYPE_UINT64:
                deserialize_field(json_obj, field_name, *reinterpret_cast<uint64_t *>(field_ptr));
                break;
            case rosidl_typesupport_introspection_cpp::ROS_TYPE_DOUBLE:
                deserialize_field(json_obj, field_name, *reinterpret_cast<double *>(field_ptr));
                break;
            case rosidl_typesupport_introspection_cpp::ROS_TYPE_MESSAGE: {
                const rosidl_typesupport_introspection_cpp::MessageMembers *nested_members =
                    static_cast<const rosidl_typesupport_introspection_cpp::MessageMembers *>(member->members_->data);
                deserialize_nested_message(field_ptr, json_obj.at(field_name), nested_members);
            }
            break;
            default:
                throw std::runtime_error("Unsupported field type for field: " + field_name);
            }
        }
    }
}

// 反序列化嵌套消息的函数
void deserialize_nested_message(void *field_ptr,
    const nlohmann::json &json_obj,
    const rosidl_typesupport_introspection_cpp::MessageMembers *members)
{
    deserialize_message_fields(field_ptr, json_obj, members);
}

// 用于从JSON反序列化任何ROS2消息的函数
template<typename T>
void deserialize_from_json(const nlohmann::json &json_obj, T &msg)
{
    std::string type_name = get_formatted_type_name(msg);

    auto introspection_library =
        rosbag2_cpp::get_typesupport_library(type_name, "rosidl_typesupport_introspection_cpp");
    if (!introspection_library)
    {
        std::cerr << "Error: Could not get typesupport library for type: " << type_name << std::endl;
        return;
    }

    auto introspection_support =
        rosbag2_cpp::get_typesupport_handle(type_name, "rosidl_typesupport_introspection_cpp", introspection_library);
    if (!introspection_support)
    {
        std::cerr << "Error: Could not get typesupport handle for type: " << type_name << std::endl;
        return;
    }

    const auto *members =
        static_cast<const rosidl_typesupport_introspection_cpp::MessageMembers *>(introspection_support->data);

    if (!members)
    {
        RCLCPP_ERROR(rclcpp::get_logger("roscpp_json"), "Invalid message members pointer.");
        return;
    }

    deserialize_message_fields(&msg, json_obj, members);
}

} // namespace rclcpp_json

#endif // RCLCPP_JSON_DESERIALIZE_H
