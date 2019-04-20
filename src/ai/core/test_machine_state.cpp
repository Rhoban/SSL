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

#include <gtest/gtest.h>

#include "machine_state.h"

#include <iostream>
#include <sstream>
#include <core/print_collection.h>

void clear(std::ostringstream& o)
{
  o.str("");
  o.clear();
}

typedef construct_machine_state_infrastructure<int, std::ostream, std::ostream> msi_int;

typedef construct_machine_state_infrastructure<std::string, std::ostream, std::ostream> msi_string;

template <typename ID>
class TestState : public machine_state::State<ID, std::ostream>
{
public:
  std::ostream& out;

  TestState(const ID& name, std::ostream& out) : machine_state::State<ID, std::ostream>(name), out(out)
  {
  }

  virtual void run(std::ostream& data, unsigned int run_number, unsigned int atomic_run_number)
  {
    auto fct = [&](std::ostream& o) {
      o << "sr " << this->name()
        << " "
           "("
        << run_number << ", " << atomic_run_number << "), ";
    };
    fct(out);
    fct(data);
  }
  virtual ~TestState()
  {
  }
};

template <typename ID>
class TestEdge : public machine_state::Edge<ID, std::ostream>
{
public:
  std::ostream& out;
  std::function<bool(void)> cond;

  TestEdge(const ID& name, const ID& origin, const ID& end, std::ostream& out,
           std::function<bool(void)> cond = []() { return true; })
    : machine_state::Edge<ID, std::ostream>(name, origin, end), out(out), cond(cond)
  {
  }

  virtual bool condition(const std::ostream& const_data, unsigned int run_number, unsigned int atomic_run_number) const
  {
    std::ostream& data = const_cast<std::ostream&>(const_data);
    auto fct = [&](std::ostream& o) {
      o << "ec " << this->name()
        << " "
           "("
        << run_number << ", " << atomic_run_number << "), ";
    };
    fct(out);
    fct(data);
    return cond();
  }

  virtual void run(std::ostream& data, unsigned int run_number, unsigned int atomic_run_number)
  {
    auto fct = [&](std::ostream& o) {
      o << "er " << this->name()
        << " "
           "("
        << run_number << ", " << atomic_run_number << "), ";
    };
    fct(out);
    fct(data);
  }
  virtual ~TestEdge()
  {
  }
};

TEST(test_machine_state, simple_cases)
{
  {
    std::ostringstream data;
    std::ostringstream out;

    msi_string::MachineState machine(data, data);

    machine.addState(std::shared_ptr<msi_string::State>(new TestState<std::string>("start", out)));

    machine.addState(std::shared_ptr<msi_string::State>(new TestState<std::string>("middle", out)));
    machine.addState(

        std::shared_ptr<msi_string::State>(new TestState<std::string>("end", out)));
    machine.addEdge(
        std::shared_ptr<msi_string::Edge>(new TestEdge<std::string>("initialisation", "start", "middle", out)));
    machine.addEdge(std::shared_ptr<msi_string::Edge>(new TestEdge<std::string>("next", "middle", "end", out)));

    machine.addInitState("start");

    EXPECT_TRUE(out.str() == "");
    EXPECT_TRUE(data.str() == "");

    machine.start();

    EXPECT_TRUE(out.str() == "");
    EXPECT_TRUE(data.str() == "");

    machine.run();

    EXPECT_TRUE(out.str() == "sr start (1, 1), "
                             "ec initialisation (1, 1), "
                             "er initialisation (1, 1), "
                             "sr middle (1, 2), "
                             "ec next (1, 2), "
                             "er next (1, 2), "
                             "sr end (1, 3), ");
    EXPECT_TRUE(data.str() == "sr start (1, 1), "
                              "ec initialisation (1, 1), "
                              "er initialisation (1, 1), "
                              "sr middle (1, 2), "
                              "ec next (1, 2), "
                              "er next (1, 2), "
                              "sr end (1, 3), ");

    clear(out);
    clear(data);

    machine.run();

    EXPECT_TRUE(out.str() == "sr end (2, 1), ");
    EXPECT_TRUE(data.str() == "sr end (2, 1), ");

    clear(out);
    clear(data);
  }
}

TEST(test_machine_state, empty_machine)
{
  {
    std::ostringstream data;
    std::ostringstream out;

    msi_string::MachineState machine(data, data);

    machine.start();

    EXPECT_TRUE(out.str() == "");
    EXPECT_TRUE(data.str() == "");

    clear(out);
    clear(data);

    machine.run();

    EXPECT_TRUE(out.str() == "");
    EXPECT_TRUE(data.str() == "");

    clear(out);
    clear(data);

    machine.run();

    EXPECT_TRUE(out.str() == "");
    EXPECT_TRUE(data.str() == "");

    clear(out);
    clear(data);
  }
}

TEST(test_machine_state, with_no_start)
{
  {
    std::ostringstream data;
    std::ostringstream out;

    msi_string::MachineState machine(data, data);

    machine.addState(std::shared_ptr<msi_string::State>(new TestState<std::string>("start", out)));

    machine.addState(std::shared_ptr<msi_string::State>(new TestState<std::string>("middle", out)));
    machine.addState(std::shared_ptr<msi_string::State>(new TestState<std::string>("end", out)));
    machine.addEdge(
        std::shared_ptr<msi_string::Edge>(new TestEdge<std::string>("initialisation", "start", "middle", out)));
    machine.addEdge(std::shared_ptr<msi_string::Edge>(new TestEdge<std::string>("next", "middle", "end", out)));

    machine.addInitState("start");

    EXPECT_TRUE(out.str() == "");
    EXPECT_TRUE(data.str() == "");

    machine.run();

    EXPECT_TRUE(out.str() == "");
    EXPECT_TRUE(data.str() == "");

    clear(out);
    clear(data);

    machine.run();

    EXPECT_TRUE(out.str() == "");
    EXPECT_TRUE(data.str() == "");

    clear(out);
    clear(data);
  }
}

TEST(test_machine_state, state_edge_data)
{
  {
    std::ostringstream edge_data;
    std::ostringstream state_data;
    std::ostringstream out;

    msi_string::MachineState machine(state_data, edge_data);

    machine.addState(std::shared_ptr<msi_string::State>(new TestState<std::string>("start", out)));

    machine.addState(std::shared_ptr<msi_string::State>(new TestState<std::string>("middle", out)));
    machine.addState(std::shared_ptr<msi_string::State>(new TestState<std::string>("end", out)));
    machine.addEdge(
        std::shared_ptr<msi_string::Edge>(new TestEdge<std::string>("initialisation", "start", "middle", out)));
    machine.addEdge(std::shared_ptr<msi_string::Edge>(new TestEdge<std::string>("next", "middle", "end", out)));

    machine.addInitState("start");

    EXPECT_TRUE(out.str() == "");
    EXPECT_TRUE(state_data.str() == "");
    EXPECT_TRUE(edge_data.str() == "");

    machine.start();

    EXPECT_TRUE(out.str() == "");
    EXPECT_TRUE(state_data.str() == "");
    EXPECT_TRUE(edge_data.str() == "");

    machine.run();

    EXPECT_TRUE(out.str() == "sr start (1, 1), "
                             "ec initialisation (1, 1), "
                             "er initialisation (1, 1), "
                             "sr middle (1, 2), "
                             "ec next (1, 2), "
                             "er next (1, 2), "
                             "sr end (1, 3), ");
    EXPECT_TRUE(edge_data.str() == "ec initialisation (1, 1), "
                                   "er initialisation (1, 1), "
                                   "ec next (1, 2), "
                                   "er next (1, 2), ");
    EXPECT_TRUE(state_data.str() == "sr start (1, 1), "
                                    "sr middle (1, 2), "
                                    "sr end (1, 3), ");

    clear(out);
    clear(state_data);
    clear(edge_data);

    machine.run();

    EXPECT_TRUE(out.str() == "sr end (2, 1), ");
    EXPECT_TRUE(edge_data.str() == "");
    EXPECT_TRUE(state_data.str() == "sr end (2, 1), ");

    clear(out);
    clear(state_data);
    clear(edge_data);
  }
}

TEST(test_machine_state, current_and_initial_states_and_are_active)
{
  {
    std::ostringstream data;
    std::ostringstream out;

    msi_string::MachineState machine(data, data);

    EXPECT_TRUE(machine.initialStates().size() == 0);
    EXPECT_TRUE(machine.currentStates().size() == 0);

    machine.addState(std::shared_ptr<msi_string::State>(new TestState<std::string>("start", out)));

    machine.addState(std::shared_ptr<msi_string::State>(new TestState<std::string>("middle", out)));
    machine.addState(std::shared_ptr<msi_string::State>(new TestState<std::string>("end", out)));
    machine.addEdge(
        std::shared_ptr<msi_string::Edge>(new TestEdge<std::string>("initialisation", "start", "middle", out)));
    machine.addEdge(std::shared_ptr<msi_string::Edge>(new TestEdge<std::string>("next", "middle", "end", out)));

    EXPECT_TRUE(machine.initialStates().size() == 0);
    EXPECT_TRUE(machine.currentStates().size() == 0);
    EXPECT_TRUE(!machine.isActive("start"));
    EXPECT_TRUE(!machine.isActive("middle"));
    EXPECT_TRUE(!machine.isActive("end"));

    EXPECT_TRUE(!machine.isInitial("start"));
    EXPECT_TRUE(!machine.isInitial("middle"));
    EXPECT_TRUE(!machine.isInitial("end"));

    machine.addInitState("start");

    EXPECT_TRUE(machine.initialStates() == std::set<std::string>({ "start" }));
    EXPECT_TRUE(machine.currentStates().size() == 0);
    EXPECT_TRUE(!machine.isActive("start"));
    EXPECT_TRUE(!machine.isActive("middle"));
    EXPECT_TRUE(!machine.isActive("end"));

    EXPECT_TRUE(machine.isInitial("start"));
    EXPECT_TRUE(!machine.isInitial("middle"));
    EXPECT_TRUE(!machine.isInitial("end"));

    machine.start();

    EXPECT_TRUE(machine.initialStates() == std::set<std::string>({ "start" }));
    EXPECT_TRUE(machine.currentStates() == std::set<std::string>({ "start" }));
    EXPECT_TRUE(machine.isActive("start"));
    EXPECT_TRUE(!machine.isActive("middle"));
    EXPECT_TRUE(!machine.isActive("end"));

    std::set<std::string> states;
    states = machine.run();

    EXPECT_TRUE(machine.initialStates() == std::set<std::string>({ "start" }));
    EXPECT_TRUE(machine.currentStates() == std::set<std::string>({ "end" }));
    EXPECT_TRUE(machine.currentStates() == states);
    EXPECT_TRUE(!machine.isActive("start"));
    EXPECT_TRUE(!machine.isActive("middle"));
    EXPECT_TRUE(machine.isActive("end"));

    states = machine.run();

    EXPECT_TRUE(machine.initialStates() == std::set<std::string>({ "start" }));
    EXPECT_TRUE(machine.currentStates() == std::set<std::string>({ "end" }));
    EXPECT_TRUE(machine.currentStates() == states);
    EXPECT_TRUE(!machine.isActive("start"));
    EXPECT_TRUE(!machine.isActive("middle"));
    EXPECT_TRUE(machine.isActive("end"));
  }
}

TEST(test_machine_state, complex_cases)
{
  {
    std::ostringstream data;
    std::ostringstream out;

    msi_int::MachineState machine(data, data);

    for (int i = 1; i < 10; i++)
    {
      machine.addState(std::shared_ptr<msi_int::State>(new TestState<int>(i, out)));
    }
    EXPECT_TRUE(machine.stateNumber() == 9);
    machine.addEdge(std::shared_ptr<msi_int::Edge>(new TestEdge<int>(1, 1, 2, out)));
    machine.addEdge(std::shared_ptr<msi_int::Edge>(new TestEdge<int>(2, 1, 3, out, []() { return false; })));
    bool button_3__4_7 = true;
    machine.addEdge(std::shared_ptr<msi_int::Edge>(new TestEdge<int>(3, 4, 7, out, [&]() { return button_3__4_7; })));
    machine.addEdge(
        std::shared_ptr<msi_int::Edge>(new TestEdge<int>(4, 4, 5, out, [&]() { return machine.isActive(2); })));
    machine.addEdge(std::shared_ptr<msi_int::Edge>(new TestEdge<int>(5, 4, 8, out)));
    machine.addEdge(std::shared_ptr<msi_int::Edge>(new TestEdge<int>(6, 7, 9, out)));
    machine.addEdge(std::shared_ptr<msi_int::Edge>(new TestEdge<int>(7, 5, 6, out)));

    bool button_8__9_4 = false;
    machine.addEdge(std::shared_ptr<msi_int::Edge>(new TestEdge<int>(8, 9, 4, out, [&]() { return button_8__9_4; })));
    EXPECT_TRUE(machine.edgeNumber() == 8);

    machine.addInitState({ 1, 4 });

    EXPECT_TRUE(machine.isInitial(1));
    EXPECT_TRUE(!machine.isInitial(2));
    EXPECT_TRUE(!machine.isInitial(3));
    EXPECT_TRUE(machine.isInitial(4));
    EXPECT_TRUE(!machine.isInitial(5));
    EXPECT_TRUE(!machine.isInitial(6));
    EXPECT_TRUE(!machine.isInitial(7));
    EXPECT_TRUE(!machine.isInitial(8));
    EXPECT_TRUE(!machine.isInitial(9));

    machine.start();

    clear(data);
    clear(out);

    machine.run();

    EXPECT_TRUE(out.str() == "sr 1 (1, 1), "
                             "sr 4 (1, 1), "
                             "ec 1 (1, 1), "
                             "ec 2 (1, 1), "
                             "ec 3 (1, 1), "
                             "ec 4 (1, 1), "
                             "ec 5 (1, 1), "
                             "er 1 (1, 1), "
                             "er 3 (1, 1), "
                             "er 5 (1, 1), "
                             "sr 2 (1, 2), "
                             "sr 7 (1, 2), "
                             "sr 8 (1, 2), "
                             "ec 6 (1, 2), "
                             "er 6 (1, 2), "
                             "sr 2 (1, 3), "
                             "sr 8 (1, 3), "
                             "sr 9 (1, 3), "
                             "ec 8 (1, 3), ");

    clear(data);
    clear(out);

    machine.run();

    EXPECT_TRUE(out.str() == "sr 2 (2, 1), "
                             "sr 8 (2, 1), "
                             "sr 9 (2, 1), "
                             "ec 8 (2, 1), ");

    clear(data);
    clear(out);

    button_8__9_4 = true;
    button_3__4_7 = false;
    machine.run();

    EXPECT_TRUE(out.str() == "sr 2 (3, 1), "
                             "sr 8 (3, 1), "
                             "sr 9 (3, 1), "
                             "ec 8 (3, 1), "
                             "er 8 (3, 1), "
                             "sr 2 (3, 2), "
                             "sr 4 (3, 2), "
                             "sr 8 (3, 2), "
                             "ec 3 (3, 2), "
                             "ec 4 (3, 2), "
                             "ec 5 (3, 2), "
                             "er 4 (3, 2), "
                             "er 5 (3, 2), "
                             "sr 2 (3, 3), "
                             "sr 5 (3, 3), "
                             "sr 8 (3, 3), "
                             "ec 7 (3, 3), "
                             "er 7 (3, 3), "
                             "sr 2 (3, 4), "
                             "sr 6 (3, 4), "
                             "sr 8 (3, 4), ");

    clear(data);
    clear(out);

    machine.run();

    EXPECT_TRUE(out.str() == "sr 2 (4, 1), "
                             "sr 6 (4, 1), "
                             "sr 8 (4, 1), ");
  }
}

TEST(test_machine_state, edge_state_number)
{
  {
    std::ostringstream data;
    std::ostringstream out;

    msi_int::MachineState machine(data, data);

    EXPECT_TRUE(machine.stateNumber() == 0);
    EXPECT_TRUE(machine.size() == machine.stateNumber());
    EXPECT_TRUE(machine.edgeNumber() == 0);
  }
  {
    std::ostringstream data;
    std::ostringstream out;

    msi_int::MachineState machine(data, data);

    for (int i = 0; i < 32; i++)
    {
      machine.addState(std::shared_ptr<msi_int::State>(new TestState<int>(i, out)));
    }
    EXPECT_TRUE(machine.stateNumber() == 32);
    EXPECT_TRUE(machine.size() == machine.stateNumber());
    for (int i = 0; i < 12; i++)
    {
      machine.addEdge(std::shared_ptr<msi_int::Edge>(new TestEdge<int>(i, i, i + 1, out)));
    }
    EXPECT_TRUE(machine.edgeNumber() == 12);
  }
}

TEST(test_machine_state, stream)
{
  {
    std::ostringstream data;
    std::ostringstream out;

    msi_string::MachineState machine(data, data);

    machine.addState(std::shared_ptr<msi_string::State>(new TestState<std::string>("start", out)));

    machine.addState(std::shared_ptr<msi_string::State>(new TestState<std::string>("middle", out)));
    machine.addState(

        std::shared_ptr<msi_string::State>(new TestState<std::string>("end", out)));
    machine.addEdge(
        std::shared_ptr<msi_string::Edge>(new TestEdge<std::string>("initialisation", "start", "middle", out)));
    machine.addEdge(std::shared_ptr<msi_string::Edge>(new TestEdge<std::string>("next", "middle", "end", out)));

    machine.addInitState("start");

    std::ostringstream description;
    description << machine;

    EXPECT_TRUE(description.str() == "States : end, middle, start, \n"
                                     "Edges : \n"
                                     " initialisation : (start, middle), \n"
                                     " next : (middle, end), \n");
  }
}

TEST(test_machine_state, to_dot)
{
  {
    std::ostringstream data;
    std::ostringstream out;

    msi_string::MachineState machine(data, data);

    machine.addState(std::shared_ptr<msi_string::State>(new TestState<std::string>("start", out)));

    machine.addState(std::shared_ptr<msi_string::State>(new TestState<std::string>("middle", out)));
    machine.addState(

        std::shared_ptr<msi_string::State>(new TestState<std::string>("end", out)));
    machine.addEdge(
        std::shared_ptr<msi_string::Edge>(new TestEdge<std::string>("initialisation", "start", "middle", out)));
    machine.addEdge(std::shared_ptr<msi_string::Edge>(new TestEdge<std::string>("next", "middle", "end", out)));

    machine.addInitState("start");

    machine.start();

    EXPECT_TRUE(machine.to_dot() == "digraph G {\n"
                                    " graph [ordering=\"out\"]\n"
                                    "\n"
                                    " v0 [label=\"end\" shape=\"oval\"];\n"
                                    " v1 [label=\"middle\" shape=\"oval\"];\n"
                                    " v2 [label=\"start\" shape=\"doubleoctagon\"style=\"filled\" "
                                    "fillcolor=\"gold\"];\n"
                                    "\n"
                                    " v2->v1 [label=\"initialisation\"];\n"
                                    " v1->v0 [label=\"next\"];\n"
                                    "}\n");

    machine.run();

    EXPECT_TRUE(machine.to_dot() == "digraph G {\n"
                                    " graph [ordering=\"out\"]\n"
                                    "\n"
                                    " v0 [label=\"end\" shape=\"oval\"style=\"filled\" fillcolor=\"gold\"];\n"
                                    " v1 [label=\"middle\" shape=\"oval\"];\n"
                                    " v2 [label=\"start\" shape=\"doubleoctagon\"];\n"
                                    "\n"
                                    " v2->v1 [label=\"initialisation\"];\n"
                                    " v1->v0 [label=\"next\"];\n"
                                    "}\n");
  }
}

TEST(test_machine_state, anonym_state_anonym_edge)
{
  {
    std::ostringstream data;
    std::ostringstream out;

    msi_string::MachineState machine(data, data);

    machine.addState("start");
    machine.addState("middle");
    machine.addState("end");

    machine.addEdge("initialisation", "start", "middle");
    machine.addEdge("next", "middle", "end");

    machine.addInitState("start");

    EXPECT_TRUE(machine.initialStates() == std::set<std::string>({ "start" }));
    EXPECT_TRUE(machine.currentStates() == std::set<std::string>());

    machine.start();

    EXPECT_TRUE(machine.initialStates() == std::set<std::string>({ "start" }));
    EXPECT_TRUE(machine.currentStates() == std::set<std::string>({ "start" }));

    machine.run();

    EXPECT_TRUE(machine.initialStates() == std::set<std::string>({ "start" }));
    EXPECT_TRUE(machine.currentStates() == std::set<std::string>({ "end" }));

    machine.run();

    EXPECT_TRUE(machine.initialStates() == std::set<std::string>({ "start" }));
    EXPECT_TRUE(machine.currentStates() == std::set<std::string>({ "end" }));
  }
  {
    std::ostringstream data;
    std::ostringstream out;

    msi_string::MachineState machine(data, data);

    machine.addState("start");
    machine.addState("middle");
    machine.addState("end");

    machine.addEdge("initialisation", "start", "middle");
    bool button_next = false;
    machine.addEdge("next", "middle", "end",
                    [&](const msi_string::EdgeData& data, unsigned int run_number, unsigned int atomic_run_number) {
                      return button_next;
                    });

    machine.addInitState("start");

    EXPECT_TRUE(machine.initialStates() == std::set<std::string>({ "start" }));
    EXPECT_TRUE(machine.currentStates() == std::set<std::string>());

    machine.start();

    EXPECT_TRUE(machine.initialStates() == std::set<std::string>({ "start" }));
    EXPECT_TRUE(machine.currentStates() == std::set<std::string>({ "start" }));

    machine.run();

    EXPECT_TRUE(machine.initialStates() == std::set<std::string>({ "start" }));
    EXPECT_TRUE(machine.currentStates() == std::set<std::string>({ "middle" }));

    machine.run();

    EXPECT_TRUE(machine.initialStates() == std::set<std::string>({ "start" }));
    EXPECT_TRUE(machine.currentStates() == std::set<std::string>({ "middle" }));

    button_next = true;
    machine.run();

    EXPECT_TRUE(machine.initialStates() == std::set<std::string>({ "start" }));
    EXPECT_TRUE(machine.currentStates() == std::set<std::string>({ "end" }));
  }
  {
    std::ostringstream data;
    std::ostringstream out;

    msi_string::MachineState machine(data, data);

    machine.addState("start",
                     [&](msi_string::StateData& data, unsigned int run_number, unsigned int atomic_run_number) {
                       data << "#sr "
                            << "START"
                            << " "
                               "("
                            << run_number << ", " << atomic_run_number << "), ";
                     });
    machine.addState("middle",
                     [&](msi_string::StateData& data, unsigned int run_number, unsigned int atomic_run_number) {
                       data << "#sr "
                            << "MIDDLE"
                            << " "
                               "("
                            << run_number << ", " << atomic_run_number << "), ";
                     });
    machine.addState("end", [&](msi_string::StateData& data, unsigned int run_number, unsigned int atomic_run_number) {
      data << "#sr "
           << "END"
           << " "
              "("
           << run_number << ", " << atomic_run_number << "), ";
    });

    machine.addEdge(
        "initialisation", "start", "middle",
        [&](const msi_string::EdgeData& const_data, unsigned int run_number, unsigned int atomic_run_number) {
          msi_string::EdgeData& data = const_cast<msi_string::EdgeData&>(const_data);
          data << "#ec "
               << "INITIALISATION"
               << " "
                  "("
               << run_number << ", " << atomic_run_number << "), ";
          return true;
        },
        [&](msi_string::EdgeData& data, unsigned int run_number, unsigned int atomic_run_number) {
          data << "#er "
               << "INITIALISATION"
               << " "
                  "("
               << run_number << ", " << atomic_run_number << "), ";
        });
    machine.addEdge(
        "next", "middle", "end",
        [&](const msi_string::EdgeData& const_data, unsigned int run_number, unsigned int atomic_run_number) {
          msi_string::EdgeData& data = const_cast<msi_string::EdgeData&>(const_data);
          data << "#ec "
               << "NEXT"
               << " "
                  "("
               << run_number << ", " << atomic_run_number << "), ";
          return true;
        },
        [&](msi_string::EdgeData& data, unsigned int run_number, unsigned int atomic_run_number) {
          data << "#er "
               << "NEXT"
               << " "
                  "("
               << run_number << ", " << atomic_run_number << "), ";
        });

    machine.addInitState("start");
    machine.start();

    clear(out);
    clear(data);

    machine.run();

    EXPECT_TRUE(data.str() == "#sr START (1, 1), "
                              "#ec INITIALISATION (1, 1), "
                              "#er INITIALISATION (1, 1), "
                              "#sr MIDDLE (1, 2), "
                              "#ec NEXT (1, 2), "
                              "#er NEXT (1, 2), "
                              "#sr END (1, 3), ");

    clear(out);
    clear(data);

    machine.run();

    EXPECT_TRUE(data.str() == "#sr END (2, 1), ");

    clear(out);
    clear(data);
  }
}

class Follower : public machine_state::MachineStateFollower<std::string, std::ostream, std::ostream>
{
public:
  std::string name;

  Follower(const std::string& name) : name(name)
  {
  }

  virtual void update(std::ostream& state_data, std::ostream& edge_data, unsigned int run_number,
                      unsigned int atomic_run_number)
  {
    auto fct = [&](std::ostream& o) {
      o << "fu " << name
        << " "
           "("
        << run_number << ", " << atomic_run_number << "), ";
    };
    fct(state_data);
  }
  virtual void atomicUpdate(std::ostream& state_data, std::ostream& edge_data, unsigned int run_number,
                            unsigned int atomic_run_number)
  {
    auto fct = [&](std::ostream& o) {
      o << "fau " << name
        << " "
           "("
        << run_number << ", " << atomic_run_number << "), ";
    };
    fct(state_data);
  }
  virtual ~Follower()
  {
  }
};

TEST(test_machine_state, register_follower)
{
  {
    std::ostringstream data;
    std::ostringstream out;

    msi_string::MachineState machine(data, data);

    machine.addState(std::shared_ptr<msi_string::State>(new TestState<std::string>("start", out)));

    machine.addState(std::shared_ptr<msi_string::State>(new TestState<std::string>("middle", out)));
    machine.addState(

        std::shared_ptr<msi_string::State>(new TestState<std::string>("end", out)));
    machine.addEdge(
        std::shared_ptr<msi_string::Edge>(new TestEdge<std::string>("initialisation", "start", "middle", out)));
    machine.addEdge(std::shared_ptr<msi_string::Edge>(new TestEdge<std::string>("next", "middle", "end", out)));

    machine.addInitState("start");

    Follower f1("f1");
    Follower f2("f2");
    machine.register_follower(f1);
    machine.register_follower(f2);

    EXPECT_TRUE(out.str() == "");
    EXPECT_TRUE(data.str() == "");

    machine.start();

    EXPECT_TRUE(out.str() == "");
    EXPECT_TRUE(data.str() == "");

    machine.run();

    EXPECT_TRUE(out.str() == "sr start (1, 1), "
                             "ec initialisation (1, 1), "
                             "er initialisation (1, 1), "
                             "sr middle (1, 2), "
                             "ec next (1, 2), "
                             "er next (1, 2), "
                             "sr end (1, 3), ");

    EXPECT_TRUE(data.str() == "fu f1 (1, 1), "
                              "fu f2 (1, 1), "
                              "fau f1 (1, 1), "
                              "fau f2 (1, 1), "
                              "sr start (1, 1), "
                              "ec initialisation (1, 1), "
                              "er initialisation (1, 1), "
                              "fau f1 (1, 2), "
                              "fau f2 (1, 2), "
                              "sr middle (1, 2), "
                              "ec next (1, 2), "
                              "er next (1, 2), "
                              "fau f1 (1, 3), "
                              "fau f2 (1, 3), "
                              "sr end (1, 3), ");

    clear(out);
    clear(data);

    machine.run();

    EXPECT_TRUE(out.str() == "sr end (2, 1), ");
    EXPECT_TRUE(data.str() == "fu f1 (2, 1), "
                              "fu f2 (2, 1), "
                              "fau f1 (2, 1), "
                              "fau f2 (2, 1), "
                              "sr end (2, 1), ");

    clear(out);
    clear(data);
  }
}

TEST(test_machine_state, rising_edge)
{
  {
    std::ostringstream data;
    std::ostringstream out;

    msi_int::MachineState machine(data, data);

    machine.addState(1);
    machine.addState(2);
    machine.addState(3);

    machine.addEdge(1, 1, 2);

    machine.addEdge(2, 2, 3,
                    msi_int::RisingEdge([&](const std::ostream& data, unsigned int run_number,
                                            unsigned int atomic_run_number) { return machine.isActive(2); },
                                        machine));

    machine.addInitState(1);

    machine.start();

    machine.run();

    EXPECT_TRUE(machine.currentStates() == std::set<int>({ 3 }));
  }
}

int main(int argc, char** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
