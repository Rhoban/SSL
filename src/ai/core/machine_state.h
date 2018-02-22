#ifndef __MACHINE_STATE__H__
#define __MACHINE_STATE__H__

#include <assert.h>
#include <string>
#include <list>
#include <map>
#include <debug.h>
#include <memory>
#include <set>

namespace machine_state {

const unsigned int CLEAR_RUN_NUMBER = 0;
const unsigned int INIT_RUN_NUMBER = 1;
const unsigned int CLEAR_ATOMIC_RUN_NUMBER = 0;
const unsigned int INIT_ATOMIC_RUN_NUMBER = 1;


template <typename ID, typename STATE_DATA> class State;
template <typename ID, typename EDGE_DATA> class Edge;

template <typename ID, typename STATE_DATA, typename EDGE_DATA> 
class MachineState;

template <typename ID, typename STATE_DATA>
std::ostream & operator<<(
    std::ostream & out, const State<ID, STATE_DATA> & state
);
template <typename ID, typename EDGE_DATA>
std::ostream & operator<<(
    std::ostream & out, const Edge<ID, EDGE_DATA> & edge
);
template <typename ID, typename STATE_DATA, typename EDGE_DATA>
std::ostream & operator<<(
    std::ostream & out, const MachineState<ID, STATE_DATA, EDGE_DATA> & machine
);


template <typename ID, typename STATE_DATA>
class State {
    private:
    ID name_state;

    public:
    State( const ID & name ):
        name_state( name )
    { }

    const ID & name() const {
        return name_state;
    }

    virtual void run(
        STATE_DATA& state_data, 
        unsigned int run_number, unsigned int atomic_run_number
    ) = 0;
    virtual ~State() { }

    friend std::ostream & operator<< <ID, STATE_DATA>(
        std::ostream & out, 
        const State<ID, STATE_DATA> & machine
    );
};

template <typename ID, typename STATE_DATA>
std::ostream & operator<<(
    std::ostream & out, const State<ID, STATE_DATA> & state
){
    out << state.name();
    return out;
};

template <typename ID, typename EDGE_DATA>
class ConditionClass {
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
    virtual bool condition(
        const EDGE_DATA& edge_data,
        unsigned int run_number, unsigned int atomic_run_number
    ) const = 0;

    virtual ~ConditionClass(){ };
};

template <typename ID, typename EDGE_DATA>
class Edge: public  ConditionClass<ID, EDGE_DATA> {
    private:
    ID name_edge;
    ID origin_state;
    ID end_state;

    public:
    Edge( 
        const ID & name,
        const ID & origin, const ID & end 
    ):
        name_edge(name), origin_state(origin), end_state(end)
    { }

    const ID & name() const {
        return name_edge;
    }
    const ID & origin() const {
        return origin_state;
    }
    const ID & end() const {
        return end_state;
    }

    virtual void run(
        EDGE_DATA & edge_data, 
        unsigned int run_number, unsigned int atomic_run_number
    ) = 0;
    virtual ~Edge() { }

    friend std::ostream & operator<< <ID, EDGE_DATA> (
        std::ostream & out, 
        const Edge<ID, EDGE_DATA> & edge
    );

};

template <typename ID, typename EDGE_DATA>
std::ostream & operator<<(
    std::ostream & out, const Edge<ID, EDGE_DATA> & edge
){
    out << edge.name() << " : (" << edge.origin() << ", " << edge.end() << ")";  
    return out;
}

template <typename ID, typename STATE_DATA>
class AnonymousState : public machine_state::State<ID, STATE_DATA> {
    private:
    std::function<
        void (STATE_DATA&, unsigned int, unsigned int)
    > run_fct;

    public:
    AnonymousState(
        const ID & id,  
        std::function<
            void (STATE_DATA&, unsigned int, unsigned int)
        > run_fct
    ):
        machine_state::State<ID, STATE_DATA>(id), run_fct(run_fct)
    { }

    virtual void run(
        STATE_DATA& data,
        unsigned int run_number, unsigned int atomic_run_number
    ){
        run_fct(data, run_number, atomic_run_number);
    }

    virtual ~AnonymousState(){}
};


template <typename ID, typename EDGE_DATA>
class AnonymousEdge : public machine_state::Edge<ID, EDGE_DATA> {
    private:
    std::function<
        bool (const EDGE_DATA&, unsigned int, unsigned int)
    > condition_fct;
    std::function<
        void (EDGE_DATA&, unsigned int, unsigned int)
    > run_fct;

    public:
    AnonymousEdge(
        const ID & id, const ID & origin, const ID & end, 
        std::function<
            bool (const EDGE_DATA&, unsigned int, unsigned int)
        > condition_fct,
        std::function<
            void (EDGE_DATA&, unsigned int, unsigned int)
        > run_fct
    ):
        machine_state::Edge<ID, EDGE_DATA>(id, origin, end),
        condition_fct(condition_fct), run_fct(run_fct) 
    {
    }

    virtual bool condition(
        const EDGE_DATA & const_data,
        unsigned int run_number, unsigned int atomic_run_number
    ) const {
        return condition_fct( const_data, run_number, atomic_run_number );
    }
    
    virtual void run(
        EDGE_DATA & data,
        unsigned int run_number, unsigned int atomic_run_number
    ){
        run_fct( data, run_number, atomic_run_number );
    }
    virtual ~AnonymousEdge(){}
};

template <typename STATE_DATA, typename EDGE_DATA>
class MachineStateFollower {
    public:
    virtual void update(
        STATE_DATA & state_data, EDGE_DATA & edge_data,
        unsigned int run_number, unsigned int atomic_run_number
    ) = 0;
    virtual void atomic_update(
        STATE_DATA & state_data, EDGE_DATA & edge_data,
        unsigned int run_number, unsigned int atomic_run_number
    ) = 0;
    virtual ~MachineStateFollower(){ }
};


template <typename ID, typename STATE_DATA, typename EDGE_DATA>
class MachineState {
    private:
        STATE_DATA & state_data;
        EDGE_DATA & edge_data;

        typedef State<ID, STATE_DATA> State_t;
        typedef Edge<ID, EDGE_DATA> Edge_t;

        std::map< ID, std::shared_ptr<Edge_t> > edges;
        std::map< ID, std::shared_ptr<State_t> > states;
        std::map< ID, std::list<ID> > adjacence;

        std::set<ID> init_states_set;
        std::set<ID> current_states_set;

        std::list<MachineStateFollower<STATE_DATA, EDGE_DATA>*> followers;

        bool debug;

        unsigned int run_number;
        unsigned int atomic_run_number;

        void increase_atomic_run_number() {
            atomic_run_number += 1;
        }
        void increase_run_number() {
            atomic_run_number = INIT_ATOMIC_RUN_NUMBER;
            run_number += 1;
        }
        
        std::set<ID> atomic_run(){
            for( MachineStateFollower<STATE_DATA, EDGE_DATA> * follower : followers ){
                follower->atomic_update(
                    state_data, edge_data,
                    run_number, atomic_run_number
                );
            }
            std::set<ID> new_states;
            for( const ID & state_name : current_states_set ){
                states.at( state_name )->run(
                    state_data, run_number, atomic_run_number
                );
            }
            std::list<ID> edges_to_run;
            for( const ID & state_name : current_states_set ){
                bool move = false;
                for(
                    const ID & edge_name : adjacence.at(state_name) 
                ){
                    std::shared_ptr<Edge_t> & edge = edges.at(edge_name);
                    const std::shared_ptr<Edge_t> & const_edge = edges.at(
                        edge_name
                    );
                    if(
                        const_edge->condition(
                            edge_data, run_number, atomic_run_number
                        )
                    ){
                        edges_to_run.push_back( edge_name );
                        new_states.insert(edge->end());
                        move = true;
                    }
                }
                if( not(move) ){
                    new_states.insert(state_name);
                }
            }
            for( const ID& id : edges_to_run ){
                edges.at(id)->run(edge_data, run_number, atomic_run_number);
            }
            increase_atomic_run_number();
            return new_states;
        }

    public:
        void add_edge( std::shared_ptr<Edge_t> edge ){
            assert( edges.find(edge->name()) == edges.end() );
            assert( states.find(edge->origin()) != states.end() );
            assert( states.find(edge->end()) != states.end() );
            
            adjacence[edge->origin()].push_back( edge->name() );
            edges[edge->name()] = edge;
        }

        void add_edge(
            const ID & id, const ID & origin, const ID & end,
            std::function<
                bool (
                    const EDGE_DATA & data, 
                    unsigned int run_number, unsigned int atomic_run_number
                )
            > condition_fct = [](
                const EDGE_DATA & data, 
                unsigned int run_number, unsigned int atomic_run_number
            ){ return true; },
            std::function<
                void (
                    EDGE_DATA & data, 
                    unsigned int run_number, unsigned int atomic_run_number
                )
            > run_fct = [](
                EDGE_DATA & data, 
                unsigned int run_number, unsigned int atomic_run_number
            ){ return ; }
        ){
            add_edge(
                std::shared_ptr< AnonymousEdge<ID, EDGE_DATA> >(
                    new AnonymousEdge<ID, EDGE_DATA>(
                        id, origin, end,
                        condition_fct, run_fct
                    )
                )
            );
        }

        void add_state( std::shared_ptr<State_t> && state ){
            assert( states.find(state->name()) == states.end() );
            assert( adjacence.find(state->name()) == adjacence.end() );

            adjacence[state->name()] = std::list<ID>();
            states[state->name()] = state;
        }

        void add_state(
            const ID & id,  
            std::function<
                void (
                    STATE_DATA & data, 
                    unsigned int run_number, unsigned int atomic_run_number
                )
            > run_fct = [](
                STATE_DATA & data, 
                unsigned int run_number, unsigned int atomic_run_number
            ){ return ; }
        ){
            add_state(
                std::shared_ptr< AnonymousState<ID, STATE_DATA> >(
                    new AnonymousState<ID, STATE_DATA>( id, run_fct )
                )
            );
        }

        void register_follower(
            MachineStateFollower<STATE_DATA, EDGE_DATA> & follower
        ){
            followers.push_back( &follower );
        }

        MachineState( STATE_DATA & state_data, EDGE_DATA & edge_data ):
            state_data(state_data), edge_data(edge_data), 
            debug(false),
            run_number(CLEAR_RUN_NUMBER),    
            atomic_run_number(CLEAR_ATOMIC_RUN_NUMBER)
        { } 

        void add_init_state( const ID & state_name ){
            assert( states.find(state_name) != states.end() );
            init_states_set.insert( state_name );
        }

        void add_init_state( const std::set<ID> & set_of_state_ids ){
            for( const ID & id : set_of_state_ids ){
                add_init_state( id );
            }
        }

        unsigned int edge_number() const {
            return edges.size();
        }

        unsigned int state_number() const {
            return states.size();
        }

        unsigned int size() const {
            return state_number();
        }

        void start(){
            current_states_set = init_states_set;
            run_number = INIT_RUN_NUMBER;   
            atomic_run_number = INIT_ATOMIC_RUN_NUMBER;
        }

        void set_debug( bool value ){
            this->debug = value;
        }

        const std::set<ID> & run(){
            for(
                MachineStateFollower<STATE_DATA, EDGE_DATA> * follower : 
                followers 
            ){
                follower->update(
                    state_data, edge_data,
                    run_number, atomic_run_number 
                );
            }
            std::set<ID> old_states;
            do {
                old_states = current_states_set;
                current_states_set = atomic_run();
            } while( old_states != current_states_set );
            
            increase_run_number();
            return current_states_set;
        }

        const std::set<ID> & current_states() const {
            return current_states_set;
        }
        
        const std::set<ID> & initial_states() const {
            return init_states_set;
        }

        std::string to_dot() const {
            std::ostringstream result;
            result <<   "digraph G {";
            result << std::endl << " graph [ordering=\"out\"]";
            result << std::endl;
            int cpt = 0;
            std::map<ID, int> states_id;
            for( auto state_asso : states ){
                const ID& id =  state_asso.first;
                states_id[id] = cpt;
                result << "\n v" << cpt 
                    << " [label=\"" << id << "\"";
                if( is_initial(id) ){
                    result << " shape=\"doublecircle\"";
                }else{
                    result << " shape=\"circle\"";
                }
                if( is_active(id) ){
                    result << "style=\"filled\" fillcolor=\"gold\"";
                }
                result << "];";
                cpt+=1;
            }
            result << std::endl;
            for( auto edge_asso : edges ){
                result << std::endl
                    << " v" << states_id.at(edge_asso.second->origin())
                    << "->"
                    << "v" << states_id.at(edge_asso.second->end())
                    << " [label=\"" << edge_asso.first << "\"];";
            }
            result << std::endl << "}";
            result << std::endl;
            return std::move(result.str());
        }

        bool is_active( const ID & state ) const {
            return current_states_set.find(state) != current_states_set.end();
        }

        bool is_initial( const ID & state ) const {
            return init_states_set.find(state) != init_states_set.end();
        }

        friend std::ostream & operator<< <ID, STATE_DATA, EDGE_DATA> (
            std::ostream & out, 
            const MachineState<ID, STATE_DATA, EDGE_DATA> & machine
        );
};

template <typename ID, typename STATE_DATA, typename EDGE_DATA>
std::ostream & operator<<(
    std::ostream & out, const MachineState<ID, STATE_DATA, EDGE_DATA> & machine
){
    out << "States : ";
    for(
        const std::pair<
            ID, std::shared_ptr< State<ID, STATE_DATA> > 
        > & state_asso : machine.states
    ){
        out << *(state_asso.second) << ", ";
    }
    out << std::endl;
    out << "Edges : " << std::endl;
    for(
        const std::pair<
           ID, std::shared_ptr< Edge<ID, EDGE_DATA> >
        > & edge_asso : 
        machine.edges
    ){
        out << " " << *(edge_asso.second) << ", " << std::endl;
    }   
    return out;
}

template <typename STATE_DATA, typename EDGE_DATA>
class RisingEdge_wrapper : public MachineStateFollower<STATE_DATA, EDGE_DATA> {
    private:
    unsigned int run_number_for_last_rising;
    unsigned int atomic_run_number_for_last_rising;

    bool last_condition_value;

    std::function<
        bool (
            const EDGE_DATA & const_data,
            unsigned int run_number, unsigned int atomic_run_number
        )
    > condition_fct;

    public:

    template <typename I>
    RisingEdge_wrapper(
        std::function<
            bool (
                const EDGE_DATA & const_data,
                unsigned int run_number, unsigned int atomic_run_number
            )
        > condition_fct,
        MachineState<I, STATE_DATA, EDGE_DATA> & machine
    ):
        run_number_for_last_rising(CLEAR_RUN_NUMBER),
        atomic_run_number_for_last_rising(CLEAR_ATOMIC_RUN_NUMBER),
        last_condition_value(false),
        condition_fct(condition_fct)
    {
        machine.register_follower( *this );
    }

    public:
    void update(
        STATE_DATA & state_data, EDGE_DATA & edge_data,
        unsigned int run_number, unsigned int atomic_run_number
    ){ };

    void atomic_update(
        STATE_DATA & state_data, EDGE_DATA & edge_data,
        unsigned int run_number, unsigned int atomic_run_number
    ){
        bool value = condition_fct(edge_data, run_number, atomic_run_number );
        bool rising_edge = (!last_condition_value) and (value);
        if( rising_edge ){
            run_number_for_last_rising = run_number;
            atomic_run_number_for_last_rising = atomic_run_number;
        } 
        last_condition_value = value;
    }

    bool condition(
        const EDGE_DATA & edge_data,
        unsigned int run_number, unsigned int atomic_run_number
    ) const {
        return (run_number_for_last_rising == run_number);
    }
};


template <typename STATE_DATA, typename EDGE_DATA>
class RisingEdge {
    private:
    std::shared_ptr< RisingEdge_wrapper<STATE_DATA, EDGE_DATA> > rising;

    public:
    template <typename I>
    RisingEdge(
        std::function<
            bool (
                const EDGE_DATA & const_data,
                unsigned int run_number, unsigned int atomic_run_number
            )
        > condition_fct,
        MachineState<I, STATE_DATA, EDGE_DATA> & machine
    ):
        rising(
            new RisingEdge_wrapper<STATE_DATA, EDGE_DATA>(
                condition_fct, machine
            )
        )
    {}

    bool operator()(
        const EDGE_DATA & edge_data,
        unsigned int run_number, unsigned int atomic_run_number
    ) const {
        return rising->condition(
            edge_data, run_number, atomic_run_number
        );
    }
};

};

template <typename ID, typename STATE_DATA, typename EDGE_DATA>
struct construct_machine_state_infrastructure {
    typedef ID Id; 
    typedef STATE_DATA StateData;
    typedef EDGE_DATA EdgeData;

    typedef machine_state::State<Id, StateData> State;
    typedef machine_state::Edge<Id, EdgeData> Edge;

    typedef machine_state::MachineState<Id, StateData, EdgeData> MachineState;

    typedef machine_state::RisingEdge<StateData, EdgeData> RisingEdge;
};

#endif
