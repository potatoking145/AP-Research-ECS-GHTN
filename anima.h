#ifndef ANIMA_anima_H__
#define ANIMA_anima_H__

#include <unordered_map>
#include <variant>
#include <string>
#include <cmath>
#include <vector>
#include <algorithm>

#include <flecs.h>

namespace anima 
{
	typedef std::variant<bool, int, std::string> worldstate_t;
	
	typedef std::unordered_map<uint_fast8_t, worldstate_t> world_t;
	
	class IAction
	{
	public:
		IAction() = default;
		~IAction() = default;
		bool virtual checkRequirments(world_t*) { return true; };
		void virtual applyResult(world_t&) { return; };
		bool virtual envoke() { return true; };
	};
	
	typedef std::vector<IAction*> task_t;
	
	class Task
	{
	private:
		task_t _actions;
	public:
#if debug == 1
		std::string name;
		Task(std::string s) : name(s) {};
#else
		Task() = default;
#endif
		~Task() = default;
		
		bool checkRequirments(world_t*);
		world_t applyResult(world_t);
		void addAction(IAction* action) { _actions.push_back(action); }
		bool executeAction(int actionIndex) { return _actions.at(actionIndex)->envoke(); };
		bool isTaskFinished(unsigned int index) { return index + 1 == _actions.size(); };
	};
	
	typedef std::vector<Task*> plan_t;
	
	class AStar
	{
	private:
		struct aStarNode
		{
			aStarNode(world_t* w, aStarNode* p, Task* t, uint16_t g, uint32_t f) : world(w), previous(p), taskTaken(t), g(g), f(f) {};
			~aStarNode() = default;
			
			world_t* world;
			aStarNode* previous;
			Task* taskTaken;
			uint16_t g;
			uint32_t f; // h + g
		};
		
		class aStarNodeCompare
		{
		public:
			bool operator() (aStarNode* lhs, aStarNode* rhs)
			{
				return lhs->f < rhs->f;
			}
		};
		
		void populatePlan(plan_t*, aStarNode*);
		
		uint8_t _maxDepth { 20 };
		bool _allowPlanAfterDepth { true };
		std::vector<world_t> _tmpWorlds;
		std::vector<aStarNode> _tmpNodes;
		std::vector<aStarNode*> _openNodes;
	public:
		AStar(uint8_t, bool);
		AStar() = default;
		~AStar() = default;
		
		void setDepth(uint8_t value);
		void setAllowPlanAfterDepth(bool value)  { _allowPlanAfterDepth = value; };
		bool constructPlan(plan_t* out, world_t* start, world_t* goal, std::vector<Task*>* tasks);
	};
	
	struct comp_AI {
		comp_AI(std::vector<Task*> tasks, world_t blackboard, world_t goal) : tasks(tasks), internalBlackBoard(blackboard), goal(goal) {};
		comp_AI() = default;
		~comp_AI() = default;
		
		bool needPlan { true };
		std::vector<Task*> tasks;
		world_t internalBlackBoard;
		world_t goal;
		plan_t plan;
		int planStep { 0 };
		int taskStep { 0 };
	};
	
	class aiSystem
	{
	private:
		flecs::system<comp_AI> _system;
	public:
		aiSystem(flecs::world*);
		~aiSystem() = default;
	};
	
} // namespace anima

#endif // ifndef ANIMA_anima_H__
