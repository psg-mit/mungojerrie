//==============================================================================
//
//  Copyright (c) 2015-
//  Authors:
//  * Joachim Klein <klein@tcs.inf.tu-dresden.de>
//  * David Mueller <david.mueller@tcs.inf.tu-dresden.de>
//
//------------------------------------------------------------------------------
//
//  This file is part of the cpphoafparser library,
//      http://automata.tools/hoa/cpphoafparser/
//
//  The cpphoafparser library is free software; you can redistribute it and/or
//  modify it under the terms of the GNU Lesser General Public
//  License as published by the Free Software Foundation; either
//  version 2.1 of the License, or (at your option) any later version.
//
//  The cpphoafparser library is distributed in the hope that it will be useful,
//  but WITHOUT ANY WARRANTY; without even the implied warranty of
//  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
//  Lesser General Public License for more details.
//
//  You should have received a copy of the GNU Lesser General Public
//  License along with this library; if not, write to the Free Software
//  Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301  USA
//
//==============================================================================

#ifndef CPPHOAFPARSER_HOACONSUMERNULL_H
#define CPPHOAFPARSER_HOACONSUMERNULL_H

#include "cpphoafparser/consumer/hoa_consumer.hh"


namespace cpphoafparser {

/**
 * A no-operation HOAConsumer.
 * Provides no-op implementations for all the functions required by the
 * HOAConsumer interface.
 */
class HOAConsumerNull : public HOAConsumer {
public:

  virtual bool parserResolvesAliases() override {
    return false;
  }

  virtual void notifyHeaderStart(const std::string& version) override {
    (void) version;
  }

  virtual void setNumberOfStates(unsigned int numberOfStates) override {
    (void) numberOfStates;
  }

  virtual void addStartStates(const int_list& stateConjunction) override {
    (void) stateConjunction;
  }

  virtual void addAlias(const std::string& name, label_expr::ptr labelExpr) override {
    (void) name;
    (void) labelExpr;
  }

  virtual void setAPs(const std::vector<std::string>& aps) override {
    (void) aps;
  }

  virtual void setAcceptanceCondition(unsigned int numberOfSets, acceptance_expr::ptr accExpr) override {
    (void) numberOfSets;
    (void) accExpr;
  }

  virtual void provideAcceptanceName(const std::string& name, const std::vector<IntOrString>& extraInfo) override {
    (void) name;
    (void) extraInfo;
  }

  virtual void setName(const std::string& name) override {
    (void) name;
  }

  virtual void setTool(const std::string& name, std::shared_ptr<std::string> version) override {
    (void) name;
    (void) version;
  }

  virtual void addProperties(const std::vector<std::string>& properties) override {
    (void) properties;
  }

  virtual void addMiscHeader(const std::string& name, const std::vector<IntOrString>& content) override {
    (void) name;
    (void) content;
  }

  virtual void notifyBodyStart() override {
  }

  virtual void addState(unsigned int id, std::shared_ptr<std::string> info, label_expr::ptr labelExpr, std::shared_ptr<int_list> accSignature) override {
    (void) id;
    (void) info;
    (void) labelExpr;
    (void) accSignature;
  }

  virtual void addEdgeImplicit(unsigned int stateId, const int_list& conjSuccessors, std::shared_ptr<int_list> accSignature) override {
    (void) stateId;
    (void) conjSuccessors;
    (void) accSignature;
  }

  virtual void addEdgeWithLabel(unsigned int stateId, label_expr::ptr labelExpr, const int_list& conjSuccessors, std::shared_ptr<int_list> accSignature) override {
    (void) stateId;
    (void) labelExpr;
    (void) conjSuccessors;
    (void) accSignature;
  }

  virtual void notifyEndOfState(unsigned int stateId) override {
    (void) stateId;
  }

  virtual void notifyEnd() override {
  }

  virtual void notifyAbort() override {
  }

  virtual void notifyWarning(const std::string& warning) override {
    (void) warning;
  }
};

}

#endif
