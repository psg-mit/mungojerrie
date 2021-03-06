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

#ifndef CPPHOAFPARSER_HOACONSUMER_H
#define CPPHOAFPARSER_HOACONSUMER_H

#include <vector>
#include <memory>

#include "cpphoafparser/ast/boolean_expression.hh"
#include "cpphoafparser/ast/atom_label.hh"
#include "cpphoafparser/ast/atom_acceptance.hh"
#include "cpphoafparser/util/int_or_string.hh"

#include "cpphoafparser/consumer/hoa_consumer_exception.hh"

namespace cpphoafparser {

/**
 * Abstract class defining the interface for consuming parse events generated by HOAParser.
 *
 * The HOAConsumer abstract class is the basic means of interacting with the parser and
 * the other infrastructure provided by cpphoafparser.
 *
 * Most of the functions in this class correspond to the various events that happen
 * during parsing of a HOA input, e.g., marking the occurrence of the various header
 * elements, the start of the automaton body, the states and edges, etc.
 * In the documentation for each function, additional information is provided whether
 * the corresponding element in a HOA input is considered optional or mandatory and
 * whether it can occur once or multiple times. For error handling, a consumer is
 * supposed to throw HOAConsumerException.
 *
 * Additionally, the consumer can indicate to the parser that aliases
 * have to be resolved before invoking any of the event methods in the consumer.
 *
 * To chain HOAConsumers, see the HOAIntermediate interface.
 */
class HOAConsumer {
public:
  /** A shared_ptr wrapping a HOAConsumer */
  typedef std::shared_ptr<HOAConsumer> ptr;
  /** A label expression */
  typedef BooleanExpression<AtomLabel> label_expr;
  /** An acceptance expression */
  typedef BooleanExpression<AtomAcceptance> acceptance_expr;
  /** A list of integers */
  typedef std::vector<unsigned int> int_list;

  /** Destructor */
  virtual ~HOAConsumer() {}

  /** 
   * This function is called by the parser to query the consumer whether aliases should be
   * resolved by the parser (return `true` or whether the consumer would like to
   * see the aliases unresolved (return `false`). This function should always return
   * a constant value.
   **/
  virtual bool parserResolvesAliases() = 0;

  /** Called by the parser for the "HOA: version" item [mandatory, once]. */
  virtual void notifyHeaderStart(const std::string& version) = 0;

  /** Called by the parser for the "States: int(numberOfStates)" item [optional, once]. */
  virtual void setNumberOfStates(unsigned int numberOfStates) = 0;

  /** 
   * Called by the parser for each "Start: state-conj" item [optional, multiple]. 
   * @param stateConjunction a list of state indices, interpreted as a conjunction
   **/
  virtual void addStartStates(const int_list& stateConjunction) = 0;

  /**
   * Called by the parser for each "Alias: alias-def" item [optional, multiple].
   * Will be called no matter the return value of `parserResolvesAliases()`.
   * 
   *  @param name the alias name (without @)
   *  @param labelExpr a boolean expression over labels
   **/
  virtual void addAlias(const std::string& name, label_expr::ptr labelExpr) = 0;

  /**
   * Called by the parser for the "AP: ap-def" item [optional, once].
   * @param aps the list of atomic propositions
   */
  virtual void setAPs(const std::vector<std::string>& aps) = 0;

  /**
   * Called by the parser for the "Acceptance: acceptance-def" item [mandatory, once].
   * @param numberOfSets the number of acceptance sets used to tag state / transition acceptance
   * @param accExpr a boolean expression over acceptance atoms
   **/
  virtual void setAcceptanceCondition(unsigned int numberOfSets, acceptance_expr::ptr accExpr) = 0;

  /** 
   * Called by the parser for each "acc-name: ..." item [optional, multiple].
   * @param name the provided name
   * @param extraInfo the additional information for this item
   * */
  virtual void provideAcceptanceName(const std::string& name, const std::vector<IntOrString>& extraInfo) = 0;

  /**
   * Called by the parser for the "name: ..." item [optional, once].
   **/
  virtual void setName(const std::string& name) = 0;

  /**
   * Called by the parser for the "tool: ..." item [optional, once].
   * @param name the tool name
   * @param version the tool version (option, empty pointer if not provided)
   **/
  virtual void setTool(const std::string& name, std::shared_ptr<std::string> version) = 0;

  /**
   * Called by the parser for the "properties: ..." item [optional, multiple].
   * @param properties a list of properties
   */
  virtual void addProperties(const std::vector<std::string>& properties) = 0;

  /** 
   * Called by the parser for each unknown header item [optional, multiple].
   * @param name the name of the header (without ':')
   * @param content a list of extra information provided by the header
   */
  virtual void addMiscHeader(const std::string& name, const std::vector<IntOrString>& content) = 0;

  /**
   * Called by the parser to notify that the BODY of the automaton has started [mandatory, once].
   */
  virtual void notifyBodyStart() = 0;

  /** 
   * Called by the parser for each "State: ..." item [multiple]. 
   * @param id the identifier for this state
   * @param info an optional string providing additional information about the state (empty pointer if not provided)
   * @param labelExpr an optional boolean expression over labels (state-labeled) (empty pointer if not provided)
   * @param accSignature an optional list of acceptance set indices (state-labeled acceptance) (empty pointer if not provided)
   */
  virtual void addState(unsigned int id, std::shared_ptr<std::string> info, label_expr::ptr labelExpr, std::shared_ptr<int_list> accSignature) = 0;

  /** 
   * Called by the parser for each implicit edge definition [multiple], i.e.,
   * where the edge label is deduced from the index of the edge.
   *
   * If the edges are provided in implicit form, after every `addState()` there should be 2^|AP| calls to
   * `addEdgeImplicit`. The corresponding boolean expression over labels / BitSet
   * can be obtained by calling BooleanExpression.fromImplicit(i-1) for the i-th call of this function per state. 
   * 
   * @param stateId the index of the 'from' state
   * @param conjSuccessors a list of successor state indices, interpreted as a conjunction 
   * @param accSignature an optional list of acceptance set indices (transition-labeled acceptance) (empty pointer if not provided)
   */
  virtual void addEdgeImplicit(unsigned int stateId, const int_list& conjSuccessors, std::shared_ptr<int_list> accSignature) = 0;

  /**
   * Called by the parser for each explicit edge definition [optional, multiple], i.e.,
   * where the label is either specified for the edge or as a state-label.
   * <br/>
   * @param stateId the index of the 'from' state
   * @param labelExpr a boolean expression over labels (empty pointer if none provided, only in case of state-labeled states)
   * @param conjSuccessors a list of successors state indices, interpreted as a conjunction 
   * @param accSignature an optional list of acceptance set indices (transition-labeled acceptance) (empty pointer if none provided)
   */
  virtual void addEdgeWithLabel(unsigned int stateId, label_expr::ptr labelExpr, const int_list& conjSuccessors, std::shared_ptr<int_list> accSignature) = 0;

  /**
   * Called by the parser to notify the consumer that the definition for state `stateId`
   * has ended [multiple].
   */
  virtual void notifyEndOfState(unsigned int stateId) = 0;

  /**
   * Called by the parser to notify the consumer that the automata definition has ended [mandatory, once].
   */
  virtual void notifyEnd() = 0;

  /**
   * Called by the parser to notify the consumer that an "ABORT" message has been encountered 
   * (at any time, indicating error, the automaton should be discarded).
   */
  virtual void notifyAbort() = 0;


  /**
   * Is called whenever a condition is encountered that merits a (non-fatal) warning.
   * The consumer is free to handle this situation as it wishes.
   */
  virtual void notifyWarning(const std::string& warning) = 0;

};

}

#endif
