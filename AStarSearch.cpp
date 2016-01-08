#include <vector>
#include <map>
#include <set>

template<class Ts, class Tc>
std::pair<std::vector<Ts>, bool> AStarSearch( 
  std::multimap<Ts, std::pair<Ts, Tc> > const& model, 
  std::map<Ts, Tc> const& heuristic, 
  Ts init_state, Ts goal_state
){
  typedef typename std::multimap<Ts, std::pair<Ts, Tc>>::iterator Action_Iter;
  class Node{
   public:
    Node(Ts const& current_state, Tc const& est_total_cost, Action_Iter const& action_iter)
      : s{current_state}, c{est_total_cost}, a{action_iter}, isinit{false} {}
    Node(Ts const& current_state, Tc const& est_total_cost)
      : s{current_state}, c{est_total_cost}, isinit{true} {}   
    ~Node(){}
    bool operator<(Node const& b) const{
      return c < b.c;
    }
    Tc getcost() const {
      return c;
    }
    Ts getstate() const {
      return s;
    }
    Action_Iter getaction() const {
      return a;
    }
    Node geparent()
    bool isinitnode() const {
      return isinit;
    }
    
   private:
    Ts const s;
    Tc const c;
    bool const isinit;
    Action_Iter const a;
  };
  std::set<Node> frontier {};
  std::set<Node> visited {};
  frontier.insert(Node{init_state, heuristic.at(init_state)});

  while(true){
    auto const& cit{*frontier.begin()};
    Node const& currnode{*cit};
    if(currnode.getstate() == goal_state){
      std::vector<Ts> r{};
      for(auto it{cit}; !it->isinitnode(); it = it->getparent())
        r.push_back(it->getstate());
        return {r, true};
    }
    //expand
    bool expandsuccess{false};
    for(auto it{model.equal_range(currnode.getstate())->first};
        it != model.equal_range(currnode.getstate())->second; 
        it++
    ){
      visited.insert(currnode);
      if(visited.find(Node{it->second.first, Tc{}}) != visited.end()){
        expandsuccess = true;
        Node nextnode{
          it->second.first, 
          it->second.second + currnode.c - heuristic.at(currnode) + heuristic.at(it->second.first), it
        };
        auto const repit{frontier.find(it->second)};
        if(repit != frontier.end() || (nextnode.getcost() < repit->getcost())){
          frontier.erase(*repit);
          frontier.insert(nextnode);
        }
      }
    }
    frontier.erase(currnode);
    if(!expandsuccess)
      return {std::vector<Ts>{}, false};
  }
}
