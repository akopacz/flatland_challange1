import numpy as np
import pickle
import networkx as nx
import sys, argparse

from flatland.envs.malfunction_generators import malfunction_from_params, MalfunctionProcessData # MalfunctionParameters
from flatland.envs.observations import GlobalObsForRailEnv
# First of all we import the Flatland rail environment
from flatland.envs.rail_env import RailEnv
from flatland.envs.rail_env import RailEnvActions
from flatland.envs.rail_generators import sparse_rail_generator
from flatland.envs.schedule_generators import sparse_schedule_generator
# We also include a renderer because we want to visualize what is going on in the environment
from flatland.utils.rendertools import RenderTool, AgentRenderVariant
from flatland.envs.agent_utils import RailAgentStatus


######### Importing custom code #########

from greedy_agent import GreedyAgent
from my_astar import AStarAgent, Node
from obs_builder import CustomObservationBuilder


######### Set parameters for rail map generation #########
width = 16 * 4 # 16 * 7  # With of map
height = 9 * 4 # 9 * 7  # Height of map
nr_trains = 10 # nr_trains = 50  # Number of trains that have an assigned task in the env
cities_in_map = 6 # 20  # Number of cities where agents can start or end
seed = 14  # Random seed
grid_distribution_of_cities = False  # Type of city distribution, if False cities are randomly placed
max_rails_between_cities = 2  # Max number of tracks allowed between cities. This is number of entry point to a city
max_rail_in_cities = 6  # Max number of parallel tracks within a city, representing a realistic trainstation


######### Initialize railway #########
rail_generator = sparse_rail_generator(max_num_cities=cities_in_map,
                                       seed=seed,
                                       grid_mode=grid_distribution_of_cities,
                                       max_rails_between_cities=max_rails_between_cities,
                                       max_rails_in_city=max_rail_in_cities,
                                       )
# Different agent types (trains) with different speeds.
speed_ration_map = {1.: 0.25,  # Fast passenger train
                    1. / 2.: 0.25,  # Fast freight train
                    1. / 3.: 0.25,  # Slow commuter train
                    1. / 4.: 0.25}  # Slow freight train
schedule_generator = sparse_schedule_generator(speed_ration_map)
stochastic_data = { "malfunction_rate":10000,  # Rate of malfunction occurence
                                        "min_duration":15,  # Minimal duration of malfunction
                                        "max_duration":50  # Max duration of malfunction
}
# observation_builder = GlobalObsForRailEnv()
observation_builder = CustomObservationBuilder()

# Construct the enviornment with the given observation, generataors, predictors, and stochastic data
local_env = RailEnv(width=width,
              height=height,
              rail_generator=rail_generator,
              schedule_generator=schedule_generator,
              number_of_agents=nr_trains,
              obs_builder_object=observation_builder,
              malfunction_generator_and_process_data=malfunction_from_params(stochastic_data),
              remove_agents_at_target=True)
observations, information = local_env.reset()

# print("observations:", observations)

# Initiate the renderer
env_renderer = RenderTool(local_env, gl="PILSVG",
                          agent_render_variant=AgentRenderVariant.ONE_STEP_BEHIND,
                          show_debug=False,
                          screen_height=1200,  # Adjust these parameters to fit your resolution
                          screen_width=1800)  # Adjust these parameters to fit your resolution


######### Get arguments of the script #########
parser=argparse.ArgumentParser()
parser.add_argument("-step", type=int,
                    help="steps")
args = parser.parse_args()


######### Custom controller: imports #########
controller = GreedyAgent(218, local_env.action_space[0])

######### Define custom controller #########
def my_controller(obs, number_of_agents, astar_paths_readable, timestamp):
    _action = {}
    grid = obs[0][0]

    # Chose an action for each agent in the environment
    for a_id in range(number_of_agents):
        status, position, direction, initial_position, target, speed = obs[a_id][1]
        action = None
        if status == RailAgentStatus.ACTIVE:
            next_cell = None
            if astar_paths_readable[a_id] is not None:
                next_cell = astar_paths_readable[a_id][agent_current_node[a_id]+1].point
            if next_cell is None or position == next_cell:
                # check if there is already someone on the route before the next intersection
                # if entered a new cell decide action
                # if next_cell is None or astar_paths_readable[a_id][agent_current_node[a_id]+1].intersection:
                if next_cell is None:
                    # plan route
                    start = Node(position, grid[position[0], position[1]], dir=direction, starting_timestamp=timestamp)
                    end = Node(target, grid[target[0], target[1]])
                    astar_paths_readable[a_id] = astar_planner.aStar(start, end, a_id, speed)
                    agent_current_node[a_id] = 0
                    if astar_paths_readable[a_id] is None:
                        # no route found 
                        # wait
                        action = 4
                        next_cell = None
                    else:
                        # update last visited nodes
                        

                        # decide which way to go next
                        from_ = astar_paths_readable[a_id][0].point
                        to = astar_paths_readable[a_id][1].point
                        action = controller.simple_act(from_, direction, to)
                        next_cell = from_
                else:
                    # follow path defined by a*
                    agent_current_node[a_id] += 1
                    # decide which way to go next
                    # from_ = next_cell
                    to = astar_paths_readable[a_id][agent_current_node[a_id] + 1].point
                    action = controller.simple_act(next_cell, direction, to)
            else:
                # (continue previous movement)
                # go forward
                action = 2
                # next_cell = AStarAgent.get_neighbors(*position)[0]
            
        elif status == RailAgentStatus.READY_TO_DEPART:
            # initializing with a do nothing movement
            action = 0 # 2
        if action is not None:
            _action.update({a_id: action})
            # astar_planner.remove_cell_from_avoid(position)
            # astar_planner.add_cell_to_avoid(astar_paths_readable[a_id][agent_current_node[a_id] + 1].point)
        else:
            _action.update({a_id: 0})
    return _action
    
    

######### Custom controller: configure paramters #########
my_grid = [[Node((i, j), local_env.rail.grid[i, j]) for j in range(local_env.rail.width)] for i in range(local_env.rail.height)]
#astar_planner = AStarAgent(my_grid, local_env.rail.width, local_env.rail.height)
astar_planner = AStarAgent(my_grid, local_env.rail.width, local_env.rail.height)
astar_paths_readable = [None for _ in range(local_env.number_of_agents)]



######### Select agents #########
# show_agents = [ 2, 6]
show_agents = range(len(local_env.agents))

# stores if agent direction is 0, 1 (or 2, 3) 
# 1 means True
agent_directions = np.zeros(local_env.number_of_agents)

# Place agent on map
action_dict = dict()
for a in show_agents:
    # action = controller.act(0)
    action = 2
    action_dict.update({a: action})
    agent_directions[a] = 1 if local_env.agents[a].direction < 2 else 0
# Do the environment step
observations, rewards, dones, information = local_env.step(action_dict)
# print("observations:", observations)

for a in show_agents:
    agent = local_env.agents[a]
    if agent.position is not None:
        # astar_planner.add_cell_to_avoid(agent.position)
        astar_planner.visited_node(agent.position, 0, a)

astar_paths_readable = [None for _ in range(local_env.number_of_agents)]
# run A* for the selected agent
#for a_id in show_agents:
#    ag = env.agents[a_id]
#    start = ag.initial_position
#    if ag.position is not None:
#        start = ag.position
#    start = Node(start, env.rail.grid[start[0], start[1]], dir=ag.direction)
#    end = Node(ag.target, env.rail.grid[ag.target[0], ag.target[1]])
#    astar_paths_readable[a_id] = astar_planner.aStar(start, end)

######### Initialize step  #########
step = 100
if args.step:
    step = args.step
    

######### Run simulation  #########
score = 0
agent_current_node = np.zeros(len(local_env.agents), dtype=int)

# Run episode
frame_step = 0
for step in range(0, step):
    # Chose an action for each agent in the environment        
        
    action = my_controller(observations, local_env.number_of_agents, astar_paths_readable, step)
    print(action)
    next_obs, all_rewards, done, info = local_env.step(action)
    
    env_renderer.render_env(show=True, show_observations=False, show_predictions=False)
    env_renderer.gl.save_image('../misc/Fames2/flatland_frame_{:04d}.png'.format(step))
    frame_step += 1
    # Update replay buffer and train agent
    for a in show_agents:
        controller.step((observations[a], action_dict[a], all_rewards[a], next_obs[a], done[a]))
        score += all_rewards[a]

    observations = next_obs.copy()

    if done['__all__']:
        break
    print('Episode: Steps {}\t Score = {}'.format(step, score))
