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

/** \file FieldValue.cpp
 *
 *
 */
#include <pprzlinkQt/FieldValue.h>
#include <iomanip>

// FIXME This should go to a SERIALISER !
std::ostream& operator<<(std::ostream& o,const pprzlink::FieldValue& v)
{
  const auto& type=v.getType();
  const auto& name=v.getName();
  if (type.isArray())
  {
    switch (type.getBaseType())
    {
      case pprzlink::BaseType::NOT_A_TYPE:
        throw std::logic_error("NOT_A_TYPE for field "+name.toStdString());
        break;
      case pprzlink::BaseType::CHAR:
      {
        std::vector<char> vec;
        v.getValue(vec);
        o << "\"";
        for (size_t i=0;i<vec.size();++i)
        {
          o << vec[i];
        }
        o << "\"";
      }
        break;
      case pprzlink::BaseType::INT8:
      {
        std::vector<int8_t> vec;
        v.getValue(vec);
        for (size_t i=0;i<vec.size();++i)
        {
          if (i!=0)
            o << ",";
          if (v.isOutputInt8AsInt())
            o << (int)vec[i];
          else
            o << vec[i];
        }
      }
        break;
      case pprzlink::BaseType::INT16:
      {
        std::vector<int16_t> vec;
        v.getValue(vec);
        for (size_t i=0;i<vec.size();++i)
        {
          if (i!=0)
            o << ",";
          o << vec[i];
        }
      }
        break;
      case pprzlink::BaseType::INT32:
      {
        std::vector<int32_t> vec;
        v.getValue(vec);
        for (size_t i=0;i<vec.size();++i)
        {
          if (i!=0)
            o << ",";
          o << vec[i];
        }
      }
        break;
      case pprzlink::BaseType::UINT8:
      {
        std::vector<uint8_t> vec;
        v.getValue(vec);
        for (size_t i=0;i<vec.size();++i)
        {
          if (i!=0)
            o << ",";
          if (v.isOutputInt8AsInt())
            o << (int)vec[i];
          else
            o << vec[i];
        }
      }
        break;
      case pprzlink::BaseType::UINT16:
      {
        std::vector<uint16_t> vec;
        v.getValue(vec);
        for (size_t i=0;i<vec.size();++i)
        {
          if (i!=0)
            o << ",";
          o << vec[i];
        }
      }
        break;
      case pprzlink::BaseType::UINT32:
      {
        std::vector<uint32_t> vec;
        v.getValue(vec);
        for (size_t i=0;i<vec.size();++i)
        {
          if (i!=0)
            o << ",";
          o << vec[i];
        }
      }
        break;
      case pprzlink::BaseType::FLOAT:
      {
        std::vector<float> vec;
        v.getValue(vec);
        for (size_t i=0;i<vec.size();++i)
        {
          if (i!=0)
            o << ",";
          o << std::fixed << std::setprecision(7) << vec[i];
        }
      }
        break;
      case pprzlink::BaseType::DOUBLE:
      {
        std::vector<double> vec;
        v.getValue(vec);
        for (size_t i=0;i<vec.size();++i)
        {
          if (i!=0)
            o << ",";
          o << std::fixed << std::setprecision(7) << vec[i];
        }
      }
        break;
      case pprzlink::BaseType::STRING:
      {
        std::vector<QString> vec;
        v.getValue(vec);
        for (size_t i=0;i<vec.size();++i)
        {
          if (i!=0)
            o << ",";
          o << vec[i].toStdString();
        }
      }
        break;
    }
  }
  else
  {
    switch (type.getBaseType())
    {
      case pprzlink::BaseType::NOT_A_TYPE:
        throw std::logic_error("NOT_A_TYPE for field "+name.toStdString());
        break;
      case pprzlink::BaseType::CHAR:
      {
        char val;
        v.getValue(val);
        o << val;
      }
        break;
      case pprzlink::BaseType::INT8:
      {
        int8_t val;
        v.getValue(val);
        if (v.isOutputInt8AsInt())
        {
          o << (int)val;
        }
        else
        {
          o << val;
        }
      }
        break;
      case pprzlink::BaseType::INT16:
      {
        int16_t val;
        v.getValue(val);
          o << val;
      }
        break;
      case pprzlink::BaseType::INT32:
      {
        int32_t val;
        v.getValue(val);
        o << val;
      }
        break;
      case pprzlink::BaseType::UINT8:
      {
        uint8_t val;
        v.getValue(val);
        if (v.isOutputInt8AsInt())
        {
          o << (int)val;
        }
        else
        {
          o << val;
        }
      }
        break;
      case pprzlink::BaseType::UINT16:
      {
        uint16_t val;
        v.getValue(val);
        o << val;
      }
        break;
      case pprzlink::BaseType::UINT32:
      {
        uint32_t val;
        v.getValue(val);
        o << val;
      }
        break;
      case pprzlink::BaseType::FLOAT:
      {
        float val;
        v.getValue(val);
        o  << std::fixed << std::setprecision(7) << val;
      }
        break;
      case pprzlink::BaseType::DOUBLE:
      {
        double val;
        v.getValue(val);
        o  << std::fixed << std::setprecision(7) << val;
      }
        break;
      case pprzlink::BaseType::STRING:
      {
        QString val;
        v.getValue(val);
        o << val.toStdString();
      }
        break;
    }
  }

  return o;
}
namespace pprzlink {
  const MessageField &FieldValue::getField() const
  {
    return field;
  }

  const FieldType &FieldValue::getType() const
  {
    return field.getType();
  }

  const QString &FieldValue::getName() const
  {
    return field.getName();
  }

  const std::any &FieldValue::getValue() const
  {
    return value;
  }

  bool FieldValue::isOutputInt8AsInt() const
  {
    return output_int8_as_int;
  }

  void FieldValue::setOutputInt8AsInt(bool outputInt8AsInt)
  {
    output_int8_as_int = outputInt8AsInt;
  }

  size_t FieldValue::addToBuffer(BytesBuffer &buffer) const
  {
    size_t initialSize=buffer.size();
    if (getType().isArray())
    {
      auto vec = std::any_cast < std::vector < std::any >> (value);
      if (getType().getArraySize()==0) // Variable length array
      {
        // TODO Check that the length is in number of elements not in bytes
        buffer.push_back(vec.size()); // Add the length of the array in number of elements
      }
      for (auto elem : vec)
      {
        switch (getType().getBaseType())
        {
          case BaseType::CHAR:
          {
            char val = std::any_cast<char>(elem);
            buffer.push_back(val);
          }
            break;
          case BaseType::INT8:
          {
            char val = std::any_cast<int8_t>(elem);
            buffer.push_back(val);
          }
            break;
          case BaseType::INT16:
          {
            char val = std::any_cast<int16_t>(elem);
            buffer.push_back(val & 0x00FFu);
            buffer.push_back((val & 0xFF00u) >> 8u);
          }
            break;
          case BaseType::INT32:
          {
            char val = std::any_cast<int32_t>(elem);
            buffer.push_back(val & 0x000000FFu);
            buffer.push_back((val & 0x0000FF00u) >> 8u);
            buffer.push_back((val & 0x00FF0000u) >> 16u);
            buffer.push_back((val & 0xFF000000u) >> 24u);
          }
            break;
          case BaseType::UINT8:
          {
            char val = std::any_cast<uint8_t>(elem);
            buffer.push_back(val);
          }
            break;
          case BaseType::UINT16:
          {
            char val = std::any_cast<uint16_t>(elem);
            buffer.push_back(val & 0x00FFu);
            buffer.push_back((val & 0xFF00u) >> 8u);
          }
            break;
          case BaseType::UINT32:
          {
            char val = std::any_cast<uint32_t>(elem);
            buffer.push_back(val & 0x000000FFu);
            buffer.push_back((val & 0x0000FF00u) >> 8u);
            buffer.push_back((val & 0x00FF0000u) >> 16u);
            buffer.push_back((val & 0xFF000000u) >> 24u);
          }
            break;
          case BaseType::FLOAT:
          {
            char val = std::any_cast<float>(elem);
            uint32_t *ptrval = (uint32_t *) &val;
            buffer.push_back(*ptrval & 0x000000FFu);
            buffer.push_back((*ptrval & 0x0000FF00u) >> 8u);
            buffer.push_back((*ptrval & 0x00FF0000u) >> 16u);
            buffer.push_back((*ptrval & 0xFF000000u) >> 24u);
          }
          case BaseType::DOUBLE:
          {
            char val = std::any_cast<double>(elem);
            uint64_t *ptrval = (uint64_t *) &val;
            buffer.push_back(*ptrval &  0x00000000000000FFu);
            buffer.push_back((*ptrval & 0x000000000000FF00u) >> 8u);
            buffer.push_back((*ptrval & 0x0000000000FF0000u) >> 16u);
            buffer.push_back((*ptrval & 0x00000000FF000000u) >> 24u);
            buffer.push_back((*ptrval & 0x000000FF00000000u) >> 32u);
            buffer.push_back((*ptrval & 0x0000FF0000000000u) >> 40u);
            buffer.push_back((*ptrval & 0x00FF000000000000u) >> 48u);
            buffer.push_back((*ptrval & 0xFF00000000000000u) >> 56u);
          }
            break;
            break;
          case BaseType::STRING:
            throw std::logic_error("Cannot add an array of STRING to a buffer for field " + getName().toStdString());
            break;
          case BaseType::NOT_A_TYPE:
            throw std::logic_error("Cannot add a field of type NOT_A_TYPE to a buffer for field " + getName().toStdString());
        }
      }
    }
    else //      if (!getType().isArray())
    {
      switch (getType().getBaseType())
      {
        case BaseType::CHAR:
        {
          char val;
          getValue(val);
          buffer.push_back(val);
        }
          break;
        case BaseType::INT8:
        {
          int8_t val;
          getValue(val);
          buffer.push_back(val);
        }
          break;
        case BaseType::INT16:
        {
          int16_t val;
          getValue(val);
          buffer.push_back(val & 0x00FFu);
          buffer.push_back((val & 0xFF00u) >> 8u);
        }
          break;
        case BaseType::INT32:
        {
          int32_t val;
          getValue(val);
          buffer.push_back(val & 0x000000FFu);
          buffer.push_back((val & 0x0000FF00u) >> 8u);
          buffer.push_back((val & 0x00FF0000u) >> 16u);
          buffer.push_back((val & 0xFF000000u) >> 24u);
        }
          break;
        case BaseType::UINT8:
        {
          uint8_t val;
          getValue(val);
          buffer.push_back(val);
        }
          break;
        case BaseType::UINT16:
        {
          uint16_t val;
          getValue(val);
          buffer.push_back(val & 0x00FFu);
          buffer.push_back((val & 0xFF00u) >> 8u);
        }
          break;
        case BaseType::UINT32:
        {
          uint32_t val;
          getValue(val);
          buffer.push_back(val & 0x000000FFu);
          buffer.push_back((val & 0x0000FF00u) >> 8u);
          buffer.push_back((val & 0x00FF0000u) >> 16u);
          buffer.push_back((val & 0xFF000000u) >> 24u);
        }
          break;
        case BaseType::FLOAT:
        {
          float val;
          getValue(val);
          uint32_t *ptrval = (uint32_t *) &val;
          buffer.push_back(*ptrval & 0x000000FFu);
          buffer.push_back((*ptrval & 0x0000FF00u) >> 8u);
          buffer.push_back((*ptrval & 0x00FF0000u) >> 16u);
          buffer.push_back((*ptrval & 0xFF000000u) >> 24u);
        }
          break;
        case BaseType::DOUBLE:
        {
          float val;
          getValue(val);
          uint64_t *ptrval = (uint64_t *) &val;
          buffer.push_back(*ptrval &  0x00000000000000FFu);
          buffer.push_back((*ptrval & 0x000000000000FF00u) >> 8u);
          buffer.push_back((*ptrval & 0x0000000000FF0000u) >> 16u);
          buffer.push_back((*ptrval & 0x00000000FF000000u) >> 24u);
          buffer.push_back((*ptrval & 0x000000FF00000000u) >> 32u);
          buffer.push_back((*ptrval & 0x0000FF0000000000u) >> 40u);
          buffer.push_back((*ptrval & 0x00FF000000000000u) >> 48u);
          buffer.push_back((*ptrval & 0xFF00000000000000u) >> 56u);
        }
          break;
        case BaseType::STRING:
        {
          // A string is encoded as a variable char array (char[])
          QString val;
          getValue(val);
          buffer.push_back(val.length());
          for (auto c: val)
          {
            buffer.push_back(c.toLatin1());
          }
        }
          break;
        case BaseType::NOT_A_TYPE:
          throw std::logic_error("Cannot add a field of type NOT_A_TYPE to a buffer for field " + getName().toStdString());
      }
    }
    return buffer.size() - initialSize;
  }

  size_t FieldValue::getByteSize() const
  {
    size_t elemSize = sizeofBaseType(getType().getBaseType());
    size_t size;
    if (getType().getBaseType() == BaseType::STRING)
    {
      elemSize = 1;
    }
    if (getType().isArray())
    {
      auto vec = std::any_cast<std::vector<std::any>>(value);
      size= elemSize * vec.size();
      if (getType().getArraySize()==0) // If variable length array add one for the length
        size++;
    }
    else if (getType().getBaseType() == BaseType::STRING)// String is considered as char[]
    {
      auto str = std::any_cast<QString>(value);
      size= elemSize*str.length() +1; // String is a vraiable length array
    }
    else
    {
      size=elemSize;
    }

    return size;
  }
}
