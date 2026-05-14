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

/** \file pprzlink_exception.h
 *
 *
 */


#ifndef PPRZLINKCPP_PPRZLINK_EXCEPTION_H
#define PPRZLINKCPP_PPRZLINK_EXCEPTION_H

#include <stdexcept>
#include <QString>

#define DECLARE_PPRZLINK_EXCEPT(a) class a : public pprzlink_exception {\
public:\
explicit a(const QString &arg) :\
  pprzlink_exception(arg) {}\
};

namespace pprzlink {
  class pprzlink_exception : public std::runtime_error {
  public:
    pprzlink_exception(const QString &arg) : runtime_error(arg.toStdString()){}
  };

  DECLARE_PPRZLINK_EXCEPT(wrong_message_format)
  DECLARE_PPRZLINK_EXCEPT(no_such_message)
  DECLARE_PPRZLINK_EXCEPT(no_such_field)
  DECLARE_PPRZLINK_EXCEPT(field_has_no_value)
  DECLARE_PPRZLINK_EXCEPT(no_such_class)
  DECLARE_PPRZLINK_EXCEPT(messages_file_not_found)
  DECLARE_PPRZLINK_EXCEPT(bad_message_file)
  DECLARE_PPRZLINK_EXCEPT(no_such_binding)
  DECLARE_PPRZLINK_EXCEPT(message_is_request)
  DECLARE_PPRZLINK_EXCEPT(message_is_not_request)
  DECLARE_PPRZLINK_EXCEPT(wrong_answer_to_request)
}
#endif //PPRZLINKCPP_PPRZLINK_EXCEPTION_H
