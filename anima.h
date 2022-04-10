#ifndef ANIMA_anima_H__
#define ANIMA_anima_H__

#include <unordered_map>
#include <variant>
#include <string>
#include <cmath>
#include <vector>
#include <algorithm>
#include <assert.h>
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
	
	class Task
	{
	private:
		std::vector<IAction> _actions;
	public:
#if debug == 1
		std::string name;
		Task(std::string s) : name(s) {};
#endif
		Task() = default;
		~Task() = default;
		
		bool checkRequirments(world_t*);
		world_t applyResult(world_t);
		void addAction(IAction* action) { _actions.push_back(*action); }
		bool executeAction(int actionIndex) { return _actions.at(actionIndex).envoke(); };
		bool isTaskFinished(unsigned int index) { return index + 1 == _actions.size(); };
	};
	
	typedef std::vector<Task*> plan_t;
	
	class AStar
	{
	private:
		struct aStarNode
		{
			aStarNode(world_t w, aStarNode* p, Task* t, uint16_t g, uint32_t f) : world(w), previous(p), taskTaken(t), g(g), f(f) {};
			~aStarNode() = default;
			
			world_t world;
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
		
		void populatePlan(plan_t*, aStarNode*) const;
		
		uint8_t _maxDepth { 20 };
		bool _allowPlanAfterDepth { true };
	public:
		AStar(uint8_t maxDepth, bool allowPlanAfterDepth)
			: _maxDepth(maxDepth),
			_allowPlanAfterDepth(allowPlanAfterDepth) {};
		AStar() = default;
		~AStar() = default;
		
		void setDepth(uint8_t value) { _maxDepth = value; };
		void setAllowPlanAfterDepth(bool value)  { _allowPlanAfterDepth = value; };
		bool constructPlan(plan_t* out, world_t* start, world_t* goal, std::vector<Task*>* tasks) const;
	};
	
	struct comp_AI
	{
		comp_AI(std::vector<Task*> tasks, world_t blackboard, world_t goal)
		: tasks(tasks), internalBlackBoard(blackboard), goal(goal) {};
		
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
	
	flecs::system<comp_AI> attatchAISystem(flecs::world*);
	
} // namespace anima

#endif // ifndef ANIMA_anima_H__
