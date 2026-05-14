/*
 * Copyright 2019 garciafa
 * This file is part of PprzLinkCPP
 *
 * PprzLinkCPP is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * PprzLinkCPP is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public License
 * along with ModemTester.  If not, see <https://www.gnu.org/licenses/>.
 *
 */

/** \file FieldValue.h
 *
 *
 */


#ifndef PPRZLINKCPP_FIELDVALUE_H
#define PPRZLINKCPP_FIELDVALUE_H

#include <pprzlinkQt/MessageField.h>
#include <any>
#include <vector>
#include <array>
#include <stdexcept>
#include <sstream>
#include <cstdint>
#include <pprzlinkQt/Device.h>

namespace pprzlink {
  /**
   * TODO
   */
  class FieldValue {
  public:
    /**
     * Default constructor of FieldValue.
     * The constructed FieldValue is not usable (used only for adding easily in containers)
     */
    FieldValue() : field("","char") {}

    /**
     * TODO
     * @tparam T
     * @param field
     * @param array
     * @param size
     */
    template<
      typename T,
      typename = typename std::enable_if<std::is_arithmetic<T>::value>::type
    >
    FieldValue(const MessageField &field, const T *array, size_t size) : field(field)
    {
      const auto &type = field.getType();
      if (type.isArray())
      {
        if (type.getArraySize() && type.getArraySize() != size)
        {
          QString sstr = "Wrong size in building value for " + field.getName() + ", got " + QString::number(size) + " / expected "
               + QString::number(type.getArraySize());
          throw std::logic_error(sstr.toStdString());
        }
        std::vector <std::any> vec;
        vec.resize(size);
        for (size_t i = 0; i < size; ++i)
        {
          vec[i] = MakeStdAny(field, array[i]);
        }
        value = std::any(vec);
      }
      else
      {
        throw std::logic_error("Cannot build scalar field from an array");
      }
    }

    /** Builds a FieldValue from a string.
     * This will build either a string type or a char array type.
     * No parsing is done for other types.
     *
     * @tparam T this will resolve to QString
     * @param field The MessageField for which the value is built
     * @param str The string to build the value from
     */
    template<
      typename T,
      typename = typename std::enable_if<std::is_same<QString, T>::value>::type
    >
    FieldValue(const MessageField &field, const T &str) : field(field)
    {
      const auto &type = field.getType();
      const auto &name = field.getName();
      if (type.getBaseType() != BaseType::STRING && !(type.isArray() && type.getBaseType() == BaseType::CHAR))
      {
        throw std::logic_error("Cannot build field " + name.toStdString() + " of type " + type.toString().toStdString() + " from string value.");
      }
      if (type.getBaseType() == BaseType::STRING)
      {
        value = std::any(QString(str));
      }
      else
      {
        // If it is a char array, treat this as any other array
        std::vector<std::any> v;
        if (type.getArraySize() && type.getArraySize() != str.size())
        {
          QString sstr = "Wrong size in building value for " + field.getName() + ", got " + QString::number(str.size()) + " / expected "
               + QString::number(type.getArraySize());
          throw std::logic_error(sstr.toStdString());
        }
        v.reserve(str.size());
        for (auto val: QString(str))
        {
          v.push_back(MakeStdAny(field, val.toLatin1()));
        }
        value = std::any(v);
      }
    }


    /**
     * TODO
     * @tparam T
     * @param field
     * @param s
     */
    template<
      typename T,
      typename = typename std::enable_if<std::is_same<char, T>::value>::type
    >
    FieldValue(const MessageField &field, const T *s) : FieldValue(field, QString(s))
    {
    }

    /**
     * TODO
     * @tparam Container
     * @tparam T
     * @param field
     * @param c
     */
    template<
      typename Container,
      typename T = typename Container::value_type,
      typename = typename std::enable_if<
        !std::is_arithmetic<Container>::value && !std::is_same<QString, Container>::value>::type
    >
    FieldValue(const MessageField &field, const Container &c) : field(field)
    {
      std::vector <std::any> v;
      const auto &type = field.getType();
      if (type.getArraySize() && type.getArraySize() != c.size())
      {
        QString sstr = "Wrong size in building value for " + field.getName() + ", got " + QString::number(c.size()) + " / expected "
             + QString::number(type.getArraySize());
        throw std::logic_error(sstr.toStdString());
      }
      v.reserve(c.size());
      for (auto val: c)
      {
        v.push_back(MakeStdAny(field, val));
      }
      value = std::any(v);
    };

    /**
     * TODO
     * @tparam T
     * @param field
     * @param v
     */
    template<
      typename T,
      typename = typename std::enable_if<std::is_arithmetic<T>::value>::type
    >
    FieldValue(const MessageField &field, T v) : field(field)
    {
      value = MakeStdAny(field, v);
    }

    /**
     * TODO
     * @tparam T
     * @tparam Size
     * @param c
     */
    template<
      typename T,
      size_t Size>
    void getValue(std::array <T, Size> &c) const
    {
      auto vec = std::any_cast < std::vector < std::any >> (value);
      for (size_t i = 0; i < c.size(); ++i)
      {
        c[i] = std::any_cast<T>(vec[i]);
      }
    }

    /**
     * TODO
     * @tparam Container
     * @tparam T
     * @param c
     */
    template<
      typename Container,
      typename T = typename Container::value_type,
      typename = typename std::enable_if<
        !std::is_arithmetic<Container>::value && !std::is_same<QString, Container>::value>::type
    >
    void getValue(Container &c) const
    {
      c.clear();
      auto vec = std::any_cast < std::vector < std::any >> (value);
      for (auto val: vec)
      {
        c.push_back(std::any_cast<T>(val));
      }
    }

    /**
     * TODO
     * @tparam T
     * @tparam E
     * @param val
     */
    template<
      typename T,
      typename E = typename std::enable_if<std::is_arithmetic<T>::value>::type
    >
    void getValue(T &val) const
    {
      val = std::any_cast<T>(value);
    }

    /**
     * TODO
     * @tparam T
     * @param val
     */
    template<typename T,
      typename = typename std::enable_if<std::is_same<QString, T>::value>::type,
      typename = typename std::enable_if<!std::is_arithmetic<T>::value>::type,
      typename = typename T::value_type>
    void getValue(T &val) const
    {
      val = std::any_cast<T>(value);
    }

    /**
     * TODO
     * @return
     */
    [[nodiscard]] const MessageField &getField() const;

    /**
     * TODO
     * @return
     */
    [[nodiscard]] const FieldType &getType() const;

    /**
     * TODO
     * @return
     */
    [[nodiscard]] const QString &getName() const;

    /**
     * TODO
     * @return
     */
    [[nodiscard]] const std::any &getValue() const;

    /**
     * TODO
     * @return
     */
    [[nodiscard]] bool isOutputInt8AsInt() const;

    /**
     * TODO
     * @param outputInt8AsInt
     */
    void setOutputInt8AsInt(bool outputInt8AsInt);

    /**
     *
     * @param buffer
     * @return The number of bytes added
     */
    size_t addToBuffer(BytesBuffer &buffer) const;

    /**
     *
     * @return the size of the field in bytes if stored in binary
     */
    size_t getByteSize() const;

  private:
    MessageField field;
    std::any value;
    bool output_int8_as_int=false;

    /**
     * TODO
     * @tparam ValueType
     * @param field
     * @param value
     * @return
     */
    template<typename ValueType>
    static std::any MakeStdAny(const MessageField &field, const ValueType &value)
    {
      auto &type = field.getType();
      auto &name = field.getName();
      switch (type.getBaseType())
      {
        case BaseType::NOT_A_TYPE:
          throw std::logic_error("Field " + name.toStdString() + " as type NOT_A_TYPE");
          break;
        case BaseType::CHAR:
          return std::any(static_cast<char>(value));
        case BaseType::INT8:
          return std::any(static_cast<int8_t>(value));
        case BaseType::INT16:
          return std::any(static_cast<int16_t>(value));
        case BaseType::INT32:
          return std::any(static_cast<int32_t>(value));
        case BaseType::UINT8:
          return std::any(static_cast<uint8_t>(value));
        case BaseType::UINT16:
          return std::any(static_cast<uint16_t>(value));
        case BaseType::UINT32:
          return std::any(static_cast<uint32_t>(value));
        case BaseType::FLOAT:
          return std::any(static_cast<float>(value));
        case BaseType::DOUBLE:
          return std::any(static_cast<double>(value));
        case BaseType::STRING:
          std::stringstream sstr;
          sstr << value;
          return std::any(QString::fromStdString(sstr.str()));
      }
      return std::any();
    }
  };
}
std::ostream& operator<<(std::ostream& o,const pprzlink::FieldValue& v);

#endif //PPRZLINKCPP_FIELDVALUE_H
