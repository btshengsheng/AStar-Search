#include <vector>
#include <map>
#include <set>
#include <queue>

//Ts: state type, Tc: cost type
template<class Ts, class Tc>
std::pair<std::vector<Ts>, bool> AStarSearch( 
  std::multimap<Ts, std::pair<Ts, Tc> > const& transition_model, 
  std::map<Ts, Tc> const& heuristic, 
  Ts init_state, Ts goal_state
){
  typedef typename std::multimap<Ts, std::pair<Ts, Tc>>::const_iterator Action_Iter;
  class Node{
   public:
    Node(Ts const& current_state, Tc const& est_total_cost, Action_Iter const& last_action)
      : s{current_state}, c{est_total_cost}, a{last_action}, isinit{false} {}
    Node(Ts const& current_state, Tc const& est_total_cost)
      : s{current_state}, c{est_total_cost}, isinit{true} {}   
    ~Node(){}
    Tc esttotalcost() const {
      return c;
    }
    Ts state() const {
      return s;
    }
    Action_Iter lastaction() const {
      return a;
    }
    bool isinitnode() const {
      return isinit;
    }
    struct costgreater{
      bool operator()(Node const& a, Node const&b){
        return a.esttotalcost() > b.esttotalcost();
      }
    };
    struct compstate{
      bool operator()(Node const& a, Node const&b){
        return a.state() < b.state();
      }
    };
            
   private:
    Ts s;
    Tc c;
    bool isinit;
    Action_Iter a;
  };

  std::priority_queue<Node, std::vector<Node>, typename Node::costgreater> frontier{};
  std::set<Node, typename Node::compstate> visited{};
  frontier.push(Node{init_state, heuristic.at(init_state)});

  while(true){
    if(frontier.empty())
      return std::pair<std::vector<Ts>, bool>{std::vector<Ts>{}, false};
      
    Node const current{frontier.top()};
    frontier.pop();
    //discard duplicates from frontier
    if(visited.find(current) != visited.end())
      continue;
    visited.insert(current);    
    //if current node is goal node, return
    if(current.state() == goal_state){
      std::vector<Ts> r{};
      Node parent{current};
      for(; !parent.isinitnode();
          parent = *(visited.find(Node{parent.lastaction()->first, Tc{}})))
        r.push_back(parent.state());
      r.push_back(parent.state());
      return std::pair<std::vector<Ts>, bool>{r, true};
    }
    //expansion
    for(Action_Iter action_iter{transition_model.equal_range(current.state()).first};
        !(action_iter == transition_model.equal_range(current.state()).second); 
        action_iter++
    ){
      Ts next_state = action_iter->second.first;
      Node next{next_state, 
                action_iter->second.second - heuristic.at(current.state()) \
                + current.esttotalcost() + heuristic.at(next_state),
                action_iter};      
      if(visited.find(next) == visited.end())
        frontier.push(next);
    }
  }
}
