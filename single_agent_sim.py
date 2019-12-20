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

from greedy_agent import GreedyAgent
from build_graph import GraphBuilder


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
observation_builder = GlobalObsForRailEnv()

# Construct the enviornment with the given observation, generataors, predictors, and stochastic data
env = RailEnv(width=width,
              height=height,
              rail_generator=rail_generator,
              schedule_generator=schedule_generator,
              number_of_agents=nr_trains,
              obs_builder_object=observation_builder,
              malfunction_generator_and_process_data=malfunction_from_params(stochastic_data),
              remove_agents_at_target=True)
env.reset()

# Initiate the renderer
env_renderer = RenderTool(env, gl="PILSVG",
                          agent_render_variant=AgentRenderVariant.ONE_STEP_BEHIND,
                          show_debug=False,
                          screen_height=1200,  # Adjust these parameters to fit your resolution
                          screen_width=1800)  # Adjust these parameters to fit your resolution


######### Get arguments of the script #########
parser=argparse.ArgumentParser()
parser.add_argument("-a", type=int,
                    help="id of the agent to be displayed")
parser.add_argument("-step", type=int,
                    help="steps")
args = parser.parse_args()

######### Select agent #########
show_agents = [0]
if args.a:
    show_agents[0] = int(args.a)
    print(show_agents)


######### Custom controller setup #########
controller = GreedyAgent(218, env.action_space[0])

# Build graph from transition map
print("\nCompute transition graph from generated rail grid.")
graph_generator = GraphBuilder(env.rail.width, env.rail.height, env.rail.grid)

# create whitelist, which cells not to remove from graph
cell_whitelist = set()
for a_id in show_agents:
    ag = env.agents[a_id]
    start = graph_generator.convert_indexes_2_node(ag.initial_position, )
    if ag.position is not None:
        start = graph_generator.convert_indexes_2_node(ag.position)
    cell_whitelist.update([start, graph_generator.convert_indexes_2_node(ag.target)])

print("white list:", cell_whitelist)

g = graph_generator.graph_from_cell_neighbors(whitelist=cell_whitelist)

# Determine optimal path for each agent with a*
astar_paths = [None for _ in range(env.number_of_agents)]
astar_paths_readable = [None for _ in range(env.number_of_agents)]
# run A* for the selected agent
for a_id in show_agents:
    ag = env.agents[a_id]
    start = graph_generator.convert_indexes_2_node(ag.initial_position)
    if ag.position is not None:
        start = graph_generator.convert_indexes_2_node(ag.position)
    end = graph_generator.convert_indexes_2_node(ag.target)
    astar_paths[a_id] = nx.astar_path(g, start, end)
    astar_paths_readable[a_id] = [graph_generator.convert_node_2_indexes(node) for node in astar_paths[a_id]]


######### Initialize step  #########
step = 200
if args.step:
    step = args.step
    

######### Run simulation  #########
action_dict = dict()
score = 0
agent_left_node = np.ones(len(env.agents), dtype=int)

# stores if agent direction is 0, 1 (or 2, 3) 
# 1 means True
agent_directions = np.zeros(env.number_of_agents)

# Place agent on map
for a in show_agents:
    action = controller.act(0)
    action_dict.update({a: action})
    agent_directions[a] = 1 if env.agents[a].direction < 2 else 0
# Do the environment step
observations, rewards, dones, information = env.step(action_dict)

# Run episode
frame_step = 0
for step in range(step):
    # Chose an action for each agent in the environment
    for a_id in show_agents:
        ag = env.agents[a_id]
        action = None
        if ag.status == RailAgentStatus.ACTIVE:
            # follow path defined by a*
            if ag.position == astar_paths_readable[a_id][agent_left_node[a_id]]:
                # arrived at a (graph node) possible intersection
                agent_left_node[a_id] += 1
                next_node = astar_paths[a_id][agent_left_node[a_id]]
                # decide which way to go next
                from_ = astar_paths[a_id][agent_left_node[a_id]-1]
                dirs = g[from_][next_node]
                if agent_directions[a_id] != 1:
                    # not the way the rail was 
                    dirs["dir0"] += 2
                    dirs["dir1"] += 2
                if dirs["dir0"] == ag.direction:
                    action = controller.change_dir_from_to(dirs["dir0"], dirs["dir1"])
                    # print(f"Agent {a_id} changed its direction from {ag.direction} to {dirs['dir1']}")
                else:
                    action = controller.change_dir_from_to(dirs["dir1"], dirs["dir0"])
                    # print(f"Agent {a_id} changed its direction from {ag.direction} to {dirs['dir0']}")
            else:
                # go forward... or check where should go
                action = 2
            
        elif ag.status == RailAgentStatus.READY_TO_DEPART:
            # initializing with a going forward movement
            action = 2
        if action is not None:
            action_dict.update({a_id: action})

    next_obs, all_rewards, done, _ = env.step(action_dict)

    env_renderer.render_env(show=True, show_observations=False, show_predictions=False)
    env_renderer.gl.save_image('../misc/Fames2/flatland_frame_{:04d}.png'.format(step))
    frame_step += 1
    # Update replay buffer and train agent
    for a in show_agents:
        controller.step((observations[a], action_dict[a], all_rewards[a], next_obs[a], done[a]))
        score += all_rewards[a]

    observations = next_obs.copy()
    for a in show_agents:
        agent_directions[a] = 1 if env.agents[a].direction < 2 else 0
    if done['__all__']:
        break
    print('Episode: Steps {}\t Score = {}'.format(step, score))