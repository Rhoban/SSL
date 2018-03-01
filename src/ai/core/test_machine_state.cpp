#include "machine_state.h"

#include <iostream>
#include <sstream>
#include <core/print_collection.h>

void clear( std::ostringstream & o ){
    o.str("");
    o.clear();
}

typedef construct_machine_state_infrastructure<int, std::ostream, std::ostream> msi_int; 

typedef construct_machine_state_infrastructure<std::string, std::ostream, std::ostream> msi_string; 

template <typename ID>
class TestState : public machine_state::State<ID, std::ostream> {
    public:
    std::ostream & out;

    TestState( const ID & name, std::ostream & out ):
        machine_state::State<ID, std::ostream>(name), out(out)
    {}
    
    virtual void run(
        std::ostream& data,
        unsigned int run_number, unsigned int atomic_run_number
    ){
        auto fct = [&]( std::ostream& o ){
            o << 
                "sr " << this->name() << " "
                "(" << 
                    run_number << ", " << atomic_run_number << 
                "), " ;
        };
        fct( out );
        fct( data );
    }
    virtual ~TestState(){}
};

template <typename ID>
class TestEdge : public machine_state::Edge<ID, std::ostream> {
    public:
    std::ostream & out;
    std::function<bool(void)> cond;

    TestEdge(
        const ID & name,
        const ID & origin, const ID & end, 
        std::ostream & out,
        std::function<bool(void)> cond = [](){ return true; }
    ):
        machine_state::Edge<ID, std::ostream>(name, origin, end), out(out),
        cond(cond)
    {}

    virtual bool condition(
        const std::ostream& const_data,
        unsigned int run_number, unsigned int atomic_run_number
    ) const {
        std::ostream & data = const_cast<std::ostream&>( const_data );
        auto fct = [&]( std::ostream& o ){
            o << 
                "ec " << this->name() << " "
                "(" << 
                    run_number << ", " << atomic_run_number << 
                "), " ;
        };
        fct( out );
        fct( data );
        return cond();
    }
    
    virtual void run(
        std::ostream& data,
        unsigned int run_number, unsigned int atomic_run_number
    ){
        auto fct = [&]( std::ostream& o ){
            o << 
                "er " << this->name() << " "
                "(" << 
                    run_number << ", " << atomic_run_number << 
                "), " ;
        };
        fct( out );
        fct( data );
    }
    virtual ~TestEdge(){}
};

void test_simple_case(){
    {
        std::ostringstream data;
        std::ostringstream out;

        msi_string::MachineState machine(data, data);

        machine.add_state(
            std::shared_ptr<msi_string::State>(
                new TestState<std::string>( "start", out )
            )
        );

        machine.add_state(
            std::shared_ptr<msi_string::State>(
                new TestState<std::string>( "middle", out )
            )
        );
        machine.add_state(
        
        std::shared_ptr<msi_string::State>(
                new TestState<std::string>( "end", out )
            )
        );
        machine.add_edge(
            std::shared_ptr<msi_string::Edge>(
                new TestEdge<std::string>( 
                    "initialisation", 
                    "start", "middle",
                    out 
                )
            )
        );
        machine.add_edge(
            std::shared_ptr<msi_string::Edge>(
                new TestEdge<std::string>( 
                    "next", 
                    "middle", "end",
                    out 
                )
            )
        );

        machine.add_init_state("start");
        
        assert( out.str() == "" );
        assert( data.str() == "" );

        machine.start();

        assert( out.str() == "" );
        assert( data.str() == "" );

        machine.run();

        assert( 
            out.str() == 
            "sr start (1, 1), "
            "ec initialisation (1, 1), "
            "er initialisation (1, 1), "
            "sr middle (1, 2), "
            "ec next (1, 2), "
            "er next (1, 2), "
            "sr end (1, 3), "
        );
        assert( 
            data.str() == 
            "sr start (1, 1), "
            "ec initialisation (1, 1), "
            "er initialisation (1, 1), "
            "sr middle (1, 2), "
            "ec next (1, 2), "
            "er next (1, 2), "
            "sr end (1, 3), "
        );

        clear( out );
        clear( data );

        machine.run();

        assert( 
            out.str() == 
            "sr end (2, 1), "
        );
        assert( 
            data.str() == 
            "sr end (2, 1), "
        );

        clear( out );
        clear( data );

    }
}

void test_empty_machine(){
    {
        std::ostringstream data;
        std::ostringstream out;

        msi_string::MachineState machine(data, data);

        machine.start();

        assert( out.str() == "" );
        assert( data.str() == "" );

        clear( out );
        clear( data );

        machine.run();

        assert( out.str() == "" );
        assert( data.str() == "" );
        
        clear( out );
        clear( data );

        machine.run();

        assert( out.str() == "" );
        assert( data.str() == "" );

        clear( out );
        clear( data );

    }
}

void test_with_no_start(){
    {
        std::ostringstream data;
        std::ostringstream out;

        msi_string::MachineState machine(data, data);

        machine.add_state(
            std::shared_ptr<msi_string::State>(
                new TestState<std::string>( "start", out )
            )
        );

        machine.add_state(
            std::shared_ptr<msi_string::State>(
                new TestState<std::string>( "middle", out )
            )
        );
        machine.add_state(
            std::shared_ptr<msi_string::State>(
                new TestState<std::string>( "end", out )
            )
        );
        machine.add_edge(
            std::shared_ptr<msi_string::Edge>(
                new TestEdge<std::string>( 
                    "initialisation", 
                    "start", "middle",
                    out 
                )
            )
        );
        machine.add_edge(
            std::shared_ptr<msi_string::Edge>(
                new TestEdge<std::string>( 
                    "next", 
                    "middle", "end",
                    out 
                )
            )
        );

        machine.add_init_state("start");
        
        assert( out.str() == "" );
        assert( data.str() == "" );

        machine.run();

        assert( out.str() == "" );
        assert( data.str() == "" );

        clear( out );
        clear( data );

        machine.run();

        assert( out.str() == "" );
        assert( data.str() == "" );

        clear( out );
        clear( data );
    }
}

void test_state_edge_data(){
    {
        std::ostringstream edge_data;
        std::ostringstream state_data;
        std::ostringstream out;

        msi_string::MachineState machine(state_data, edge_data);

        machine.add_state(
            std::shared_ptr<msi_string::State>(
                new TestState<std::string>( "start", out )
            )
        );

        machine.add_state(
            std::shared_ptr<msi_string::State>(
                new TestState<std::string>( "middle", out )
            )
        );
        machine.add_state(
            std::shared_ptr<msi_string::State>(
                new TestState<std::string>( "end", out )
            )
        );
        machine.add_edge(
            std::shared_ptr<msi_string::Edge>(
                new TestEdge<std::string>( 
                    "initialisation", 
                    "start", "middle",
                    out 
                )
            )
        );
        machine.add_edge(
            std::shared_ptr<msi_string::Edge>(
                new TestEdge<std::string>( 
                    "next", 
                    "middle", "end",
                    out 
                )
            )
        );

        machine.add_init_state("start");
        
        assert( out.str() == "" );
        assert( state_data.str() == "" );
        assert( edge_data.str() == "" );

        machine.start();

        assert( out.str() == "" );
        assert( state_data.str() == "" );
        assert( edge_data.str() == "" );

        machine.run();

        assert( 
            out.str() == 
            "sr start (1, 1), "
            "ec initialisation (1, 1), "
            "er initialisation (1, 1), "
            "sr middle (1, 2), "
            "ec next (1, 2), "
            "er next (1, 2), "
            "sr end (1, 3), "
        );
        assert( 
            edge_data.str() == 
            "ec initialisation (1, 1), "
            "er initialisation (1, 1), "
            "ec next (1, 2), "
            "er next (1, 2), "
        );
        assert( 
            state_data.str() == 
            "sr start (1, 1), "
            "sr middle (1, 2), "
            "sr end (1, 3), "
        );

        clear( out );
        clear( state_data );
        clear( edge_data );

        machine.run();

        assert( 
            out.str() == 
            "sr end (2, 1), "
        );
        assert( 
            edge_data.str() == 
            ""
        );
        assert( 
            state_data.str() == 
            "sr end (2, 1), "
        );

        clear( out );
        clear( state_data );
        clear( edge_data );

    }
}

void test_current_and_initial_states_and_is_active(){
    {
        std::ostringstream data;
        std::ostringstream out;

        msi_string::MachineState machine(data, data);

        assert( machine.initial_states().size() == 0 );
        assert( machine.current_states().size() == 0 );

        machine.add_state(
            std::shared_ptr<msi_string::State>(
                new TestState<std::string>( "start", out )
            )
        );

        machine.add_state(
            std::shared_ptr<msi_string::State>(
                new TestState<std::string>( "middle", out )
            )
        );
        machine.add_state(
            std::shared_ptr<msi_string::State>(
                new TestState<std::string>( "end", out )
            )
        );
        machine.add_edge(
            std::shared_ptr<msi_string::Edge>(
                new TestEdge<std::string>( 
                    "initialisation", 
                    "start", "middle",
                    out 
                )
            )
        );
        machine.add_edge(
            std::shared_ptr<msi_string::Edge>(
                new TestEdge<std::string>( 
                    "next", 
                    "middle", "end",
                    out 
                )
            )
        );


        assert( machine.initial_states().size() == 0 );
        assert( machine.current_states().size() == 0 );
        assert( !machine.is_active( "start" ) );
        assert( !machine.is_active( "middle" ) );
        assert( !machine.is_active( "end" ) );

        assert( !machine.is_initial( "start" ) );
        assert( !machine.is_initial( "middle" ) );
        assert( !machine.is_initial( "end" ) );

        machine.add_init_state("start");

        assert( machine.initial_states() == std::set<std::string>({"start"}) );
        assert( machine.current_states().size() == 0 );
        assert( !machine.is_active( "start" ) );
        assert( !machine.is_active( "middle" ) );
        assert( !machine.is_active( "end" ) );

        assert( machine.is_initial( "start" ) );
        assert( !machine.is_initial( "middle" ) );
        assert( !machine.is_initial( "end" ) );
        
        machine.start();

        assert( machine.initial_states() == std::set<std::string>({"start"}) );
        assert( 
            machine.current_states() == std::set<std::string>({"start"})
        );
        assert( machine.is_active( "start" ) );
        assert( !machine.is_active( "middle" ) );
        assert( !machine.is_active( "end" ) );

        std::set<std::string> states;
        states = machine.run();

        assert( machine.initial_states() == std::set<std::string>({"start"}) );
        assert( 
            machine.current_states() == std::set<std::string>({"end"})
        );
        assert( machine.current_states() == states );
        assert( !machine.is_active( "start" ) );
        assert( !machine.is_active( "middle" ) );
        assert( machine.is_active( "end" ) );

        states = machine.run();

        assert( machine.initial_states() == std::set<std::string>({"start"}) );
        assert( 
            machine.current_states() == std::set<std::string>({"end"})
        );
        assert( machine.current_states() == states );
        assert( !machine.is_active( "start" ) );
        assert( !machine.is_active( "middle" ) );
        assert( machine.is_active( "end" ) );
    }
}

void test_complex_case(){
    {
        std::ostringstream data;
        std::ostringstream out;

        msi_int::MachineState machine(data, data);

        for( int i=1; i<10; i++ ){ 
            machine.add_state(
                std::shared_ptr<msi_int::State>(
                    new TestState<int>( i, out )
                )
            );
        }
        assert( machine.state_number() == 9 );
        machine.add_edge(
            std::shared_ptr<msi_int::Edge>(
                new TestEdge<int>( 1,  1, 2, out )
            )
        );
        machine.add_edge(
            std::shared_ptr<msi_int::Edge>(
                new TestEdge<int>( 2,  1, 3, out, [](){return false;} )
            )
        );
        bool button_3__4_7 = true;
        machine.add_edge(
            std::shared_ptr<msi_int::Edge>(
                new TestEdge<int>( 3,  4, 7, out, [&](){ return button_3__4_7;} )
            )
        );
        machine.add_edge(
            std::shared_ptr<msi_int::Edge>(
                new TestEdge<int>( 
                    4,  4, 5, out, [&](){return machine.is_active(2);} 
                )
            )
        );
        machine.add_edge(
            std::shared_ptr<msi_int::Edge>(
                new TestEdge<int>( 5,  4, 8, out )
            )
        );
        machine.add_edge(
            std::shared_ptr<msi_int::Edge>(
                new TestEdge<int>( 6,  7, 9, out )
            )
        );
        machine.add_edge(
            std::shared_ptr<msi_int::Edge>(
                new TestEdge<int>( 7,  5, 6, out )
            )
        );

        bool button_8__9_4 = false;
        machine.add_edge(
            std::shared_ptr<msi_int::Edge>(
                new TestEdge<int>( 8,  9, 4, out, [&](){ return button_8__9_4;} )
            )
        );
        assert( machine.edge_number() == 8 );

        machine.add_init_state({ 1, 4 });
       
        assert(  machine.is_initial( 1 ) );
        assert( !machine.is_initial( 2 ) );
        assert( !machine.is_initial( 3 ) );
        assert(  machine.is_initial( 4 ) );
        assert( !machine.is_initial( 5 ) );
        assert( !machine.is_initial( 6 ) );
        assert( !machine.is_initial( 7 ) );
        assert( !machine.is_initial( 8 ) );
        assert( !machine.is_initial( 9 ) );
 
        machine.start();


        clear( data );
        clear( out );

        machine.run();

        assert( 
            out.str() == 
            "sr 1 (1, 1), "
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
            "ec 8 (1, 3), "
        );


        clear( data );
        clear( out );

        machine.run();

        assert( 
            out.str() == 
            "sr 2 (2, 1), "
            "sr 8 (2, 1), "
            "sr 9 (2, 1), "
            "ec 8 (2, 1), "
        );

        clear( data );
        clear( out );

        button_8__9_4 = true;
        button_3__4_7 = false;
        machine.run();

        assert( 
            out.str() == 
            "sr 2 (3, 1), "
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
            "sr 8 (3, 4), "
        );

        clear( data );
        clear( out );


        machine.run();

        assert( 
            out.str() == 
            "sr 2 (4, 1), "
            "sr 6 (4, 1), "
            "sr 8 (4, 1), "
        );

    }
}

void test_edge_state_number(){
    {
        std::ostringstream data;
        std::ostringstream out;

        msi_int::MachineState machine(data, data);

        assert( machine.state_number() == 0 );
        assert( machine.size() == machine.state_number() );
        assert( machine.edge_number() == 0 );
    }
    {
        std::ostringstream data;
        std::ostringstream out;

        msi_int::MachineState machine(data, data);

        for( int i=0; i<32; i++ ){ 
            machine.add_state(
                std::shared_ptr<msi_int::State>(
                    new TestState<int>( i, out )
                )
            );
        }
        assert( machine.state_number() == 32 );
        assert( machine.size() == machine.state_number() );
        for( int i=0; i<12; i++ ){ 
            machine.add_edge(
                std::shared_ptr<msi_int::Edge>(
                    new TestEdge<int>( i,  i, i+1, out )
                )
            );
        }
        assert( machine.edge_number() == 12 );
    }
}


void test_stream(){
    {
        std::ostringstream data;
        std::ostringstream out;

        msi_string::MachineState machine(data, data);

        machine.add_state(
            std::shared_ptr<msi_string::State>(
                new TestState<std::string>( "start", out )
            )
        );

        machine.add_state(
            std::shared_ptr<msi_string::State>(
                new TestState<std::string>( "middle", out )
            )
        );
        machine.add_state(
        
        std::shared_ptr<msi_string::State>(
                new TestState<std::string>( "end", out )
            )
        );
        machine.add_edge(
            std::shared_ptr<msi_string::Edge>(
                new TestEdge<std::string>( 
                    "initialisation", 
                    "start", "middle",
                    out 
                )
            )
        );
        machine.add_edge(
            std::shared_ptr<msi_string::Edge>(
                new TestEdge<std::string>( 
                    "next", 
                    "middle", "end",
                    out 
                )
            )
        );

        machine.add_init_state("start");

        std::ostringstream description;
        description << machine;

        assert( 
            description.str()==
            "States : end, middle, start, \n"
            "Edges : \n"
            " initialisation : (start, middle), \n"
            " next : (middle, end), \n"
        );
    }
}

void test_to_dot(){
    {
        std::ostringstream data;
        std::ostringstream out;

        msi_string::MachineState machine(data, data);

        machine.add_state(
            std::shared_ptr<msi_string::State>(
                new TestState<std::string>( "start", out )
            )
        );

        machine.add_state(
            std::shared_ptr<msi_string::State>(
                new TestState<std::string>( "middle", out )
            )
        );
        machine.add_state(
        
        std::shared_ptr<msi_string::State>(
                new TestState<std::string>( "end", out )
            )
        );
        machine.add_edge(
            std::shared_ptr<msi_string::Edge>(
                new TestEdge<std::string>( 
                    "initialisation", 
                    "start", "middle",
                    out 
                )
            )
        );
        machine.add_edge(
            std::shared_ptr<msi_string::Edge>(
                new TestEdge<std::string>( 
                    "next", 
                    "middle", "end",
                    out 
                )
            )
        );

        machine.add_init_state("start");

        machine.start();

        assert(
            machine.to_dot() ==
            "digraph G {\n"
            " graph [ordering=\"out\"]\n"
            "\n"
            " v0 [label=\"end\" shape=\"oval\"];\n"
            " v1 [label=\"middle\" shape=\"oval\"];\n"
            " v2 [label=\"start\" shape=\"doubleoctagon\"style=\"filled\" fillcolor=\"gold\"];\n"
            "\n"
            " v2->v1 [label=\"initialisation\"];\n"
            " v1->v0 [label=\"next\"];\n"
            "}\n"
        );

        machine.run();

        assert(
            machine.to_dot() ==
            "digraph G {\n"
            " graph [ordering=\"out\"]\n"
            "\n"
            " v0 [label=\"end\" shape=\"oval\"style=\"filled\" fillcolor=\"gold\"];\n"
            " v1 [label=\"middle\" shape=\"oval\"];\n"
            " v2 [label=\"start\" shape=\"doubleoctagon\"];\n"
            "\n"
            " v2->v1 [label=\"initialisation\"];\n"
            " v1->v0 [label=\"next\"];\n"
            "}\n"
        );
    }
}

void test_anonymstate_anonymedge(){
    {
        std::ostringstream data;
        std::ostringstream out;

        msi_string::MachineState machine(data, data);

        machine.add_state( "start" );
        machine.add_state( "middle" );
        machine.add_state( "end" );
        
        machine.add_edge("initialisation", "start", "middle");
        machine.add_edge("next", "middle", "end");

        machine.add_init_state("start");

        assert( machine.initial_states() == std::set<std::string>({"start"}) );
        assert( machine.current_states() == std::set<std::string>() );

        machine.start();

        assert( machine.initial_states() == std::set<std::string>({"start"}) );
        assert( machine.current_states() == std::set<std::string>({"start"}) );

        machine.run();

        assert( machine.initial_states() == std::set<std::string>({"start"}) );
        assert( machine.current_states() == std::set<std::string>({"end"}) );

        machine.run();

        assert( machine.initial_states() == std::set<std::string>({"start"}) );
        assert( machine.current_states() == std::set<std::string>({"end"}) );
    }
    {
        std::ostringstream data;
        std::ostringstream out;

        msi_string::MachineState machine(data, data);

        machine.add_state( "start" );
        machine.add_state( "middle" );
        machine.add_state( "end" );
        
        machine.add_edge("initialisation", "start", "middle");
        bool button_next = false;
        machine.add_edge(
            "next", "middle", "end",
            [&](
                const msi_string::EdgeData & data, 
                unsigned int run_number, unsigned int atomic_run_number
            ){ return button_next; }
        );

        machine.add_init_state("start");

        assert( machine.initial_states() == std::set<std::string>({"start"}) );
        assert( machine.current_states() == std::set<std::string>() );

        machine.start();

        assert( machine.initial_states() == std::set<std::string>({"start"}) );
        assert( machine.current_states() == std::set<std::string>({"start"}) );

        machine.run();

        assert( machine.initial_states() == std::set<std::string>({"start"}) );
        assert( machine.current_states() == std::set<std::string>({"middle"}) );

        machine.run();

        assert( machine.initial_states() == std::set<std::string>({"start"}) );
        assert( machine.current_states() == std::set<std::string>({"middle"}) );
        
        button_next = true;
        machine.run();

        assert( machine.initial_states() == std::set<std::string>({"start"}) );
        assert( machine.current_states() == std::set<std::string>({"end"}) );
    }
    {
        std::ostringstream data;
        std::ostringstream out;

        msi_string::MachineState machine(data, data);

        machine.add_state(
            "start",
            [&](
                msi_string::StateData & data, 
                unsigned int run_number, unsigned int atomic_run_number
            ){
                data << 
                    "#sr " << "START" << " "
                    "(" << 
                        run_number << ", " << atomic_run_number << 
                    "), " ;
            }
        );
        machine.add_state(
            "middle",
            [&](
                msi_string::StateData & data, 
                unsigned int run_number, unsigned int atomic_run_number
            ){
                data << 
                    "#sr " << "MIDDLE" << " "
                    "(" << 
                        run_number << ", " << atomic_run_number << 
                    "), " ;
            }
        );
        machine.add_state(
            "end",
            [&](
                msi_string::StateData & data, 
                unsigned int run_number, unsigned int atomic_run_number
            ){
                data << 
                    "#sr " << "END" << " "
                    "(" << 
                        run_number << ", " << atomic_run_number << 
                    "), " ;
            }
        );
        
        machine.add_edge(
            "initialisation", "start", "middle",
            [&](
                const msi_string::EdgeData & const_data, 
                unsigned int run_number, unsigned int atomic_run_number
            ){
                msi_string::EdgeData & data = const_cast<msi_string::EdgeData&>(
                    const_data
                );
                data << 
                    "#ec " << "INITIALISATION" << " "
                    "(" << 
                        run_number << ", " << atomic_run_number << 
                    "), " ;
                return true;
            },
            [&](
                msi_string::EdgeData & data, 
                unsigned int run_number, unsigned int atomic_run_number
            ){
                data << 
                    "#er " << "INITIALISATION" << " "
                    "(" << 
                        run_number << ", " << atomic_run_number << 
                    "), " ;
            }
        );
        machine.add_edge(
            "next", "middle", "end",
            [&](
                const msi_string::EdgeData & const_data, 
                unsigned int run_number, unsigned int atomic_run_number
            ){
                msi_string::EdgeData & data = const_cast<msi_string::EdgeData&>(
                    const_data
                );
                data << 
                    "#ec " << "NEXT" << " "
                    "(" << 
                        run_number << ", " << atomic_run_number << 
                    "), " ;
                return true;
            },
            [&](
                msi_string::EdgeData & data, 
                unsigned int run_number, unsigned int atomic_run_number
            ){
                data << 
                    "#er " << "NEXT" << " "
                    "(" << 
                        run_number << ", " << atomic_run_number << 
                    "), " ;
            }
        );

        machine.add_init_state("start");
        machine.start();

        clear( out );
        clear( data );

        machine.run();

        assert( 
            data.str() == 
            "#sr START (1, 1), "
            "#ec INITIALISATION (1, 1), "
            "#er INITIALISATION (1, 1), "
            "#sr MIDDLE (1, 2), "
            "#ec NEXT (1, 2), "
            "#er NEXT (1, 2), "
            "#sr END (1, 3), "
        );

        clear( out );
        clear( data );

        machine.run();


        assert( 
            data.str() == 
            "#sr END (2, 1), "
        );

        clear( out );
        clear( data );
    }


}

class Follower : public machine_state::MachineStateFollower<std::ostream, std::ostream> {
    public:
        std::string name;
    
    Follower(const std::string & name):name(name){ }
    
    virtual void update(
        std::ostream & state_data, std::ostream & edge_data,
        unsigned int run_number, unsigned int atomic_run_number
    ){
        auto fct = [&]( std::ostream& o ){
            o << 
                "fu " << name << " "
                "(" << 
                    run_number << ", " << atomic_run_number << 
                "), " ;
        };
        fct( state_data );
    }
    virtual void atomic_update(
        std::ostream & state_data, std::ostream & edge_data,
        unsigned int run_number, unsigned int atomic_run_number
    ){
        auto fct = [&]( std::ostream& o ){
            o << 
                "fau " << name << " "
                "(" << 
                    run_number << ", " << atomic_run_number << 
                "), " ;
        };
        fct( state_data );
    }
    virtual ~Follower(){ }
};

void test_register_follower(){
    {
        std::ostringstream data;
        std::ostringstream out;

        msi_string::MachineState machine(data, data);

        machine.add_state(
            std::shared_ptr<msi_string::State>(
                new TestState<std::string>( "start", out )
            )
        );

        machine.add_state(
            std::shared_ptr<msi_string::State>(
                new TestState<std::string>( "middle", out )
            )
        );
        machine.add_state(
        
        std::shared_ptr<msi_string::State>(
                new TestState<std::string>( "end", out )
            )
        );
        machine.add_edge(
            std::shared_ptr<msi_string::Edge>(
                new TestEdge<std::string>( 
                    "initialisation", 
                    "start", "middle",
                    out 
                )
            )
        );
        machine.add_edge(
            std::shared_ptr<msi_string::Edge>(
                new TestEdge<std::string>( 
                    "next", 
                    "middle", "end",
                    out 
                )
            )
        );

        machine.add_init_state("start");

        Follower f1("f1");
        Follower f2("f2");
        machine.register_follower( f1 );
        machine.register_follower( f2 );
        
        assert( out.str() == "" );
        assert( data.str() == "" );

        machine.start();

        assert( out.str() == "" );
        assert( data.str() == "" );

        machine.run();


        assert( 
            out.str() == 
            "sr start (1, 1), "
            "ec initialisation (1, 1), "
            "er initialisation (1, 1), "
            "sr middle (1, 2), "
            "ec next (1, 2), "
            "er next (1, 2), "
            "sr end (1, 3), "
        );
        
        assert( 
            data.str() == 
            "fu f1 (1, 1), "
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
            "sr end (1, 3), "
        );

        clear( out );
        clear( data );

        machine.run();

        assert( 
            out.str() == 
            "sr end (2, 1), "
        );
        assert( 
            data.str() == 
            "fu f1 (2, 1), "
            "fu f2 (2, 1), "
            "fau f1 (2, 1), "
            "fau f2 (2, 1), "
            "sr end (2, 1), "
        );

        clear( out );
        clear( data );

    }
}

void test_rising_edge(){
    {
        std::ostringstream data;
        std::ostringstream out;

        msi_int::MachineState machine(data, data);

        machine.add_state(1);
        machine.add_state(2);
        machine.add_state(3);

        machine.add_edge( 1, 1, 2 );

        machine.add_edge(
            2, 2, 3,
            msi_int::RisingEdge(
                [&](
                    const std::ostream& data,
                    unsigned int run_number, unsigned int atomic_run_number
                ){ 
                    return machine.is_active(2); 
                },
                machine
            )
        );

        machine.add_init_state(1);

        machine.start();

        machine.run();

        assert( machine.current_states() == std::set<int>({3}) );        
    }
}


int main(){

    test_simple_case();
    test_empty_machine();
    test_with_no_start();
    test_state_edge_data();
    test_current_and_initial_states_and_is_active();
    test_complex_case();
    test_edge_state_number();
    test_stream();
    test_to_dot();
    test_anonymstate_anonymedge();
    test_register_follower();
    
    test_rising_edge();

    return 0;
}
