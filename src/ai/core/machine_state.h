/*
    This file is part of SSL.

    Copyright 2018 Boussicault Adrien (adrien.boussicault@u-bordeaux.fr)

    SSL is free software: you can redistribute it and/or modify
    it under the terms of the GNU Lesser General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    SSL is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU Lesser General Public License for more details.

    You should have received a copy of the GNU Lesser General Public License
    along with SSL.  If not, see <http://www.gnu.org/licenses/>.
*/

#ifndef __MACHINE_STATE__H__
#define __MACHINE_STATE__H__

#include <assert.h>
#include <string>
#include <list>
#include <map>
#include <debug.h>
#include <memory>
#include <set>
#include <sstream>
#include <functional>
#include <fstream>

namespace machine_state
{
const unsigned int CLEAR_RUN_NUMBER = 0;
const unsigned int INIT_RUN_NUMBER = 1;
const unsigned int CLEAR_ATOMIC_RUN_NUMBER = 0;
const unsigned int INIT_ATOMIC_RUN_NUMBER = 1;

template <typename ID, typename STATE_DATA>
class State;
template <typename ID, typename EDGE_DATA>
class Edge;

template <typename ID, typename STATE_DATA, typename EDGE_DATA>
class MachineState;

template <typename ID, typename STATE_DATA>
std::ostream& operator<<(std::ostream& out, const State<ID, STATE_DATA>& state);
template <typename ID, typename EDGE_DATA>
std::ostream& operator<<(std::ostream& out, const Edge<ID, EDGE_DATA>& edge);
template <typename ID, typename STATE_DATA, typename EDGE_DATA>
std::ostream& operator<<(std::ostream& out, const MachineState<ID, STATE_DATA, EDGE_DATA>& machine);

template <typename ID, typename STATE_DATA>
class State
{
private:
  ID name_state_;

public:
  State(const ID& name) : name_state_(name)
  {
  }

  const ID& name() const
  {
    return name_state_;
  }

  virtual void run(STATE_DATA& state_data, unsigned int run_number, unsigned int atomic_run_number) = 0;
  virtual ~State()
  {
  }

  friend std::ostream& operator<<<ID, STATE_DATA>(std::ostream& out, const State<ID, STATE_DATA>& machine);
};

template <typename ID, typename STATE_DATA>
std::ostream& operator<<(std::ostream& out, const State<ID, STATE_DATA>& state)
{
  out << state.name();
  return out;
};

template <typename ID, typename EDGE_DATA>
class ConditionClass
{
public:
  /*
   * The execution of condition(...) should not depend from
   * the execution order of all the condition(...) function
   * of all the edges.
   *
   * Thati's why, the parameter edge_data is const.
   *
   * If you want, you can modify edge_data, just make e
   * const_cast<EDGE_DATA>(edge_data) in your condition
   * function.
   * However, you have to check that your code respect
   * the previous remark.
   */
  virtual bool condition(const EDGE_DATA& edge_data, unsigned int run_number, unsigned int atomic_run_number) const = 0;

  virtual ~ConditionClass(){};
};

template <typename ID, typename EDGE_DATA>
class Edge : public ConditionClass<ID, EDGE_DATA>
{
private:
  ID name_edge_;
  ID origin_state_;
  ID end_state_;

public:
  Edge(const ID& name, const ID& origin, const ID& end) : name_edge_(name), origin_state_(origin), end_state_(end)
  {
  }

  const ID& name() const
  {
    return name_edge_;
  }
  const ID& origin() const
  {
    return origin_state_;
  }
  const ID& end() const
  {
    return end_state_;
  }

  virtual void run(EDGE_DATA& edge_data, unsigned int run_number, unsigned int atomic_run_number) = 0;
  virtual ~Edge()
  {
  }

  friend std::ostream& operator<<<ID, EDGE_DATA>(std::ostream& out, const Edge<ID, EDGE_DATA>& edge);
};

template <typename ID, typename EDGE_DATA>
std::ostream& operator<<(std::ostream& out, const Edge<ID, EDGE_DATA>& edge)
{
  out << edge.name() << " : (" << edge.origin() << ", " << edge.end() << ")";
  return out;
}

template <typename ID, typename STATE_DATA>
class AnonymousState : public machine_state::State<ID, STATE_DATA>
{
private:
  std::function<void(STATE_DATA&, unsigned int, unsigned int)> run_fct;

public:
  AnonymousState(const ID& id, std::function<void(STATE_DATA&, unsigned int, unsigned int)> run_fct)
    : machine_state::State<ID, STATE_DATA>(id), run_fct(run_fct)
  {
  }

  virtual void run(STATE_DATA& data, unsigned int run_number, unsigned int atomic_run_number)
  {
    run_fct(data, run_number, atomic_run_number);
  }

  virtual ~AnonymousState()
  {
  }
};

template <typename ID, typename EDGE_DATA>
class AnonymousEdge : public machine_state::Edge<ID, EDGE_DATA>
{
private:
  std::function<bool(const EDGE_DATA&, unsigned int, unsigned int)> condition_fct_;
  std::function<void(EDGE_DATA&, unsigned int, unsigned int)> run_fct_;

public:
  AnonymousEdge(const ID& id, const ID& origin, const ID& end,
                std::function<bool(const EDGE_DATA&, unsigned int, unsigned int)> condition_fct,
                std::function<void(EDGE_DATA&, unsigned int, unsigned int)> run_fct)
    : machine_state::Edge<ID, EDGE_DATA>(id, origin, end), condition_fct_(condition_fct), run_fct_(run_fct)
  {
  }

  virtual bool condition(const EDGE_DATA& const_data, unsigned int run_number, unsigned int atomic_run_number) const
  {
    return condition_fct_(const_data, run_number, atomic_run_number);
  }

  virtual void run(EDGE_DATA& data, unsigned int run_number, unsigned int atomic_run_number)
  {
    run_fct_(data, run_number, atomic_run_number);
  }
  virtual ~AnonymousEdge()
  {
  }
};

template <typename ID, typename STATE_DATA, typename EDGE_DATA>
class MachineStateFollower
{
public:
  virtual void update(STATE_DATA& state_data, EDGE_DATA& edge_data, unsigned int run_number,
                      unsigned int atomic_run_number) = 0;
  virtual void atomicUpdate(STATE_DATA& state_data, EDGE_DATA& edge_data, unsigned int run_number,
                            unsigned int atomic_run_number) = 0;
  virtual void edgeRun(ID edge_id, STATE_DATA& state_data, EDGE_DATA& edge_data, unsigned int run_number,
                       unsigned int atomic_run_number){};
  virtual void stateRun(ID state_id, STATE_DATA& state_data, EDGE_DATA& edge_data, unsigned int run_number,
                        unsigned int atomic_run_number){};
  virtual ~MachineStateFollower()
  {
  }
};

template <typename ID, typename STATE_DATA, typename EDGE_DATA>
struct EdgeFollower : public MachineStateFollower<ID, STATE_DATA, EDGE_DATA>
{
  typedef std::function<void(ID edge_id, STATE_DATA& state_data, EDGE_DATA& edge_data, unsigned int run_number,
                             unsigned int atomic_run_number)>
      EdgeRun;

  EdgeRun edge_run_fct;

  EdgeFollower(EdgeRun edge_run_fct) : edge_run_fct(edge_run_fct)
  {
  }

  void update(STATE_DATA& state_data, EDGE_DATA& edge_data, unsigned int run_number, unsigned int atomic_run_number)
  {
  }
  void atomicUpdate(STATE_DATA& state_data, EDGE_DATA& edge_data, unsigned int run_number,
                    unsigned int atomic_run_number)
  {
  }
  void edgeRun(ID edge_id, STATE_DATA& state_data, EDGE_DATA& edge_data, unsigned int run_number,
               unsigned int atomic_run_number)
  {
    edge_run_fct(edge_id, state_data, edge_data, run_number, atomic_run_number);
  };
  void stateRun(ID state_id, STATE_DATA& state_data, EDGE_DATA& edge_data, unsigned int run_number,
                unsigned int atomic_run_number){};
};

template <typename ID, typename STATE_DATA, typename EDGE_DATA>
class MachineState
{
private:
  STATE_DATA& state_data_;
  EDGE_DATA& edge_data_;

  typedef State<ID, STATE_DATA> State_t;
  typedef Edge<ID, EDGE_DATA> Edge_t;

  std::map<ID, std::shared_ptr<Edge_t> > edges_;
  std::map<ID, std::shared_ptr<State_t> > states_;
  std::map<ID, std::list<ID> > adjacence_;

  std::set<ID> init_states_set_;
  std::set<ID> current_states_set_;

  std::list<MachineStateFollower<ID, STATE_DATA, EDGE_DATA>*> followers_;

  bool debug_;

  unsigned int run_number_;
  unsigned int atomic_run_number_;

  void increaseAtomicRunNumber()
  {
    atomic_run_number_ += 1;
  }
  void increaseRunNumber()
  {
    atomic_run_number_ = INIT_ATOMIC_RUN_NUMBER;
    run_number_ += 1;
  }

  std::set<ID> atomicRun()
  {
    for (MachineStateFollower<ID, STATE_DATA, EDGE_DATA>* follower : followers_)
    {
      follower->atomicUpdate(state_data_, edge_data_, run_number_, atomic_run_number_);
    }
    std::set<ID> new_states;
    for (const ID& state_name : current_states_set_)
    {
      states_.at(state_name)->run(state_data_, run_number_, atomic_run_number_);
      for (MachineStateFollower<ID, STATE_DATA, EDGE_DATA>* follower : followers_)
      {
        follower->stateRun(state_name, state_data_, edge_data_, run_number_, atomic_run_number_);
      }
    }
    std::list<ID> edges_to_run;
    for (const ID& state_name : current_states_set_)
    {
      bool move = false;
      for (const ID& edge_name : adjacence_.at(state_name))
      {
        std::shared_ptr<Edge_t>& edge = edges_.at(edge_name);
        const std::shared_ptr<Edge_t>& const_edge = edges_.at(edge_name);
        if (const_edge->condition(edge_data_, run_number_, atomic_run_number_))
        {
          edges_to_run.push_back(edge_name);
          new_states.insert(edge->end());
          move = true;
        }
      }
      if (not(move))
      {
        new_states.insert(state_name);
      }
    }
    for (const ID& id : edges_to_run)
    {
      edges_.at(id)->run(edge_data_, run_number_, atomic_run_number_);
      for (MachineStateFollower<ID, STATE_DATA, EDGE_DATA>* follower : followers_)
      {
        follower->edgeRun(id, state_data_, edge_data_, run_number_, atomic_run_number_);
      }
    }
    increaseAtomicRunNumber();
    return new_states;
  }

public:
  MachineState& addEdge(std::shared_ptr<Edge_t> edge)
  {
    assert(edges_.find(edge->name()) == edges_.end());
    assert(states_.find(edge->origin()) != states_.end());
    assert(states_.find(edge->end()) != states_.end());

    adjacence_[edge->origin()].push_back(edge->name());
    edges_[edge->name()] = edge;
    return *this;
  }

  MachineState&
  addEdge(const ID& id, const ID& origin, const ID& end,
          std::function<bool(const EDGE_DATA& data, unsigned int run_number, unsigned int atomic_run_number)>
              condition_fct =
                  [](const EDGE_DATA& data, unsigned int run_number, unsigned int atomic_run_number) { return true; },
          std::function<void(EDGE_DATA& data, unsigned int run_number, unsigned int atomic_run_number)> run_fct =
              [](EDGE_DATA& data, unsigned int run_number, unsigned int atomic_run_number) { return; })
  {
    return addEdge(std::shared_ptr<AnonymousEdge<ID, EDGE_DATA> >(
        new AnonymousEdge<ID, EDGE_DATA>(id, origin, end, condition_fct, run_fct)));
  }

  MachineState& addState(std::shared_ptr<State_t>&& state)
  {
    assert(states_.find(state->name()) == states_.end());
    assert(adjacence_.find(state->name()) == adjacence_.end());

    adjacence_[state->name()] = std::list<ID>();
    states_[state->name()] = state;
    return *this;
  }

  MachineState&
  addState(const ID& id,
           std::function<void(STATE_DATA& data, unsigned int run_number, unsigned int atomic_run_number)> run_fct =
               [](STATE_DATA& data, unsigned int run_number, unsigned int atomic_run_number) { return; })
  {
    return addState(std::shared_ptr<AnonymousState<ID, STATE_DATA> >(new AnonymousState<ID, STATE_DATA>(id, run_fct)));
  }

  MachineState& registerFollower(MachineStateFollower<ID, STATE_DATA, EDGE_DATA>& follower)
  {
    followers_.push_back(&follower);
    return *this;
  }

private:
  std::list<EdgeFollower<ID, STATE_DATA, EDGE_DATA> > edge_followers_;

public:
  void executeAtEachEdge(std::function<void(ID edge_id, STATE_DATA& state_data, EDGE_DATA& edge_data,
                                            unsigned int run_number, unsigned int atomic_run_number)>
                             edge_run)
  {
    edge_followers_.push_back(EdgeFollower<ID, STATE_DATA, EDGE_DATA>(edge_run));
    registerFollower(edge_followers_.back());
  }

  MachineState(STATE_DATA& state_data, EDGE_DATA& edge_data)
    : state_data_(state_data)
    , edge_data_(edge_data)
    , debug_(false)
    , run_number_(CLEAR_RUN_NUMBER)
    , atomic_run_number_(CLEAR_ATOMIC_RUN_NUMBER)
  {
  }

  MachineState& addInitState(const ID& state_name)
  {
    assert(states_.find(state_name) != states_.end());
    init_states_set_.insert(state_name);
    return *this;
  }

  MachineState& addInitState(const std::set<ID>& set_of_state_ids)
  {
    for (const ID& id : set_of_state_ids)
    {
      addInitState(id);
    }
    return *this;
  }

  unsigned int edgeNumber() const
  {
    return edges_.size();
  }

  const Edge_t& edge(const ID& edge_id) const
  {
    assert(edges_.find(edge_id) != edges_.end());
    return *edges_.at(edge_id);
  }

  unsigned int stateNumber() const
  {
    return states_.size();
  }

  unsigned int size() const
  {
    return stateNumber();
  }

  void start()
  {
    current_states_set_ = init_states_set_;
    run_number_ = INIT_RUN_NUMBER;
    atomic_run_number_ = INIT_ATOMIC_RUN_NUMBER;
  }

  void setDebug(bool value)
  {
    this->debug_ = value;
  }

  const std::set<ID>& run()
  {
    for (MachineStateFollower<ID, STATE_DATA, EDGE_DATA>* follower : followers_)
    {
      follower->update(state_data_, edge_data_, run_number_, atomic_run_number_);
    }
    std::set<ID> old_states;
    do
    {
      old_states = current_states_set_;
      current_states_set_ = atomicRun();
    } while (old_states != current_states_set_);

    increaseRunNumber();
    return current_states_set_;
  }

  const std::set<ID>& currentStates() const
  {
    return current_states_set_;
  }

  const std::set<ID>& initialStates() const
  {
    return init_states_set_;
  }

  unsigned int getRunNumber() const
  {
    return run_number_;
  }

  std::string toDot() const
  {
    std::ostringstream result;
    result << "digraph G {";
    result << std::endl << " graph [ordering=\"out\"]";
    result << std::endl;
    int cpt = 0;
    std::map<ID, int> states_id;
    for (auto state_asso : states_)
    {
      const ID& id = state_asso.first;
      states_id[id] = cpt;
      result << "\n v" << cpt << " [label=\"" << id << "\"";
      if (isInitial(id))
      {
        result << " shape=\"doubleoctagon\"";
      }
      else
      {
        result << " shape=\"oval\"";
      }
      if (isActive(id))
      {
        result << "style=\"filled\" fillcolor=\"gold\"";
      }
      result << "];";
      cpt += 1;
    }
    result << std::endl;
    for (auto edge_asso : edges_)
    {
      result << std::endl
             << " v" << states_id.at(edge_asso.second->origin()) << "->"
             << "v" << states_id.at(edge_asso.second->end()) << " [label=\"" << edge_asso.first << "\"];";
    }
    result << std::endl << "}";
    result << std::endl;
    return std::move(result.str());
  }

  bool exportToFile(const std::string& path)
  {
    std::ofstream file(path);
    if (!file.is_open())
      return false;
    file << toDot();
    file.close();
    return true;
  }

  bool isActive(const ID& state) const
  {
    return current_states_set_.find(state) != current_states_set_.end();
  }

  bool isInitial(const ID& state) const
  {
    return init_states_set_.find(state) != init_states_set_.end();
  }

  friend std::ostream& operator<<<ID, STATE_DATA, EDGE_DATA>(std::ostream& out,
                                                             const MachineState<ID, STATE_DATA, EDGE_DATA>& machine);
};

template <typename ID, typename STATE_DATA, typename EDGE_DATA>
std::ostream& operator<<(std::ostream& out, const MachineState<ID, STATE_DATA, EDGE_DATA>& machine)
{
  out << "States : ";
  for (const std::pair<ID, std::shared_ptr<State<ID, STATE_DATA> > >& state_asso : machine.states_)
  {
    out << *(state_asso.second) << ", ";
  }
  out << std::endl;
  out << "Edges : " << std::endl;
  for (const std::pair<ID, std::shared_ptr<Edge<ID, EDGE_DATA> > >& edge_asso : machine.edges_)
  {
    out << " " << *(edge_asso.second) << ", " << std::endl;
  }
  return out;
}

template <typename ID, typename STATE_DATA, typename EDGE_DATA>
class RisingEdgeWrapper : public MachineStateFollower<ID, STATE_DATA, EDGE_DATA>
{
private:
  unsigned int run_number_for_last_rising_;
  unsigned int atomic_run_number_for_last_rising_;

  bool last_condition_value_;

  std::function<bool(const EDGE_DATA& const_data, unsigned int run_number, unsigned int atomic_run_number)>
      condition_fct_;

public:
  RisingEdgeWrapper(
      std::function<bool(const EDGE_DATA& const_data, unsigned int run_number, unsigned int atomic_run_number)>
          condition_fct,
      MachineState<ID, STATE_DATA, EDGE_DATA>& machine)
    : run_number_for_last_rising_(CLEAR_RUN_NUMBER)
    , atomic_run_number_for_last_rising_(CLEAR_ATOMIC_RUN_NUMBER)
    , last_condition_value_(false)
    , condition_fct_(condition_fct)
  {
    machine.registerFollower(*this);
  }

public:
  void update(STATE_DATA& state_data, EDGE_DATA& edge_data, unsigned int run_number, unsigned int atomic_run_number){};

  void atomicUpdate(STATE_DATA& state_data, EDGE_DATA& edge_data, unsigned int run_number,
                    unsigned int atomic_run_number)
  {
    bool value = condition_fct_(edge_data, run_number, atomic_run_number);
    bool rising_edge = (!last_condition_value_) and (value);
    if (rising_edge)
    {
      run_number_for_last_rising_ = run_number;
      atomic_run_number_for_last_rising_ = atomic_run_number;
    }
    last_condition_value_ = value;
  }

  bool condition(const EDGE_DATA& edge_data, unsigned int run_number, unsigned int atomic_run_number) const
  {
    return (run_number_for_last_rising_ == run_number);
  }
};

template <typename ID, typename STATE_DATA, typename EDGE_DATA>
class RisingEdge
{
private:
  std::shared_ptr<RisingEdgeWrapper<ID, STATE_DATA, EDGE_DATA> > rising_;

public:
  RisingEdge(std::function<bool(const EDGE_DATA& const_data, unsigned int run_number, unsigned int atomic_run_number)>
                 condition_fct,
             MachineState<ID, STATE_DATA, EDGE_DATA>& machine)
    : rising_(new RisingEdgeWrapper<ID, STATE_DATA, EDGE_DATA>(condition_fct, machine))
  {
  }

  bool operator()(const EDGE_DATA& edge_data, unsigned int run_number, unsigned int atomic_run_number) const
  {
    return rising_->condition(edge_data, run_number, atomic_run_number);
  }
};

};  // namespace machine_state

template <typename ID, typename STATE_DATA, typename EDGE_DATA>
struct construct_machine_state_infrastructure
{
  typedef ID Id;
  typedef STATE_DATA StateData;
  typedef EDGE_DATA EdgeData;

  typedef machine_state::State<Id, StateData> State;
  typedef machine_state::Edge<Id, EdgeData> Edge;

  typedef machine_state::MachineState<Id, StateData, EdgeData> MachineState;

  typedef machine_state::RisingEdge<Id, StateData, EdgeData> RisingEdge;

  typedef machine_state::MachineStateFollower<Id, StateData, EdgeData> MachineStateFollower;
};

#endif
