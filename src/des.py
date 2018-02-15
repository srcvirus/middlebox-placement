import argparse
import heapq
import networkx as nx
import os
import re
import subprocess


class Event:
    def __init__(self, ts, etype, flow_id):
        self.ts = ts
        self.etype = etype
        self.flow_id = flow_id

    def __lt__(self, e):
        return self.ts < e.ts

    def debug_string(self):
        return "ts = " + str(self.ts) + ", etype = " + self.etype + ", flow_id = " + self.flow_id

class TrafficRequest:
    def __init__(self, flow_id, src, dst, bw, mbox_seq):
        self.flow_id = flow_id
        self.src = src
        self.dst = dst
        self.bw = bw
        self.mbox_seq = mbox_seq

def execute_one_experiment(executable, sn_topology_file, flows_file,
        mbox_spec_file, log_prefix, traffic_index):
    process = subprocess.Popen([executable, '--per_core_cost=0', '--per_bit_transit_cost=0', 
                                '--topology_file=' + sn_topology_file, '--traffic_request_file=' +
                                flows_file, '--algorithm=viterbi', '--traffic_index=' + traffic_index,
                                '--log_prefix=' + log_prefix, '--middlebox_spec_file=' + mbox_spec_file], 
                                stdout=subprocess.PIPE, stderr = subprocess.PIPE, shell=False)
    for line in process.stdout:
        print line.strip("\r\n")
    for line in process.stderr:
        print line.strip("\r\n")

def load_mbox_specs(mbox_spec_file):
    ret = dict()
    with open(mbox_spec_file, "r") as f:
        for line in f:
            tokens = line.strip("\n\r").split(",")
            ret[int(tokens[0])] = int(tokens[1])
    return ret

def load_traffic_requests(traffic_request_file):
    ret = []
    with open(traffic_request_file, "r") as f:
        for line in f:
            tokens = line.strip("\n\r").split(",")
            flow_id = int(tokens[0])
            src = int(tokens[1])
            dst = int(tokens[2])
            bw = int(tokens[3])
            mbox_seq = tokens[4:]
            ret.append(TrafficRequest(flow_id, src, dst, bw, mbox_seq))
    return ret

def load_csv_graph(topology_file):
    g = nx.Graph()
    with open(topology_file, "r") as f:
        line = f.readline().strip("\n\r")
        # print line
        tokens = line.split(",")
        num_nodes, num_edges = int(tokens[0]), int(tokens[1])
        # print num_nodes
        for i in range(0, num_nodes):
            line = f.readline().strip("\n\r")
            tokens = line.split(",")
            g.add_node(int(tokens[0]), {"cpu": int(tokens[1]), "bw":int(tokens[2])})
        for line in f:
            tokens = line.split(",")
            lid, u, v, bw = int(tokens[0]), int(tokens[1]), int(tokens[2]), long(tokens[3])
            if u > v:
                u, v = v, u
            g.add_edge(u, v, {'bw':bw})
    return g

def write_csv_graph(g, topology_file):
    with open(topology_file, "w") as f:
        f.write(str(g.number_of_nodes()) + "," + str(g.number_of_edges()) + "\n")
        nodes = g.nodes(data = "all")
        for node in nodes:
            nid, cpu, sw = node[0], node[1]['cpu'], node[1]['bw']
            f.write(",".join([str(nid), str(cpu), str(sw)]) + "\n")
        edges = g.edges()
        lid = 0
        for edge in edges:
            bw = g.get_edge_data(edge[0], edge[1])['bw']
            f.write(",".join([str(lid),str(edge[0]), str(edge[1]), str(bw)]) + "\n")
            lid += 1

def write_util_matrix(sn, util_matrix, out_file):
    with open(out_file, "w") as f:
        for (key, value) in util_matrix.iteritems():
            util = float(value / (value + float(sn.get_edge_data(key[0], key[1])['bw'])))
            if util > 0:
                f.write(",".join([str(key[0]), str(key[1]), str(util)]) + "\n")

def write_server_util(sn, node_util, out_file):
    with open(out_file, "w") as f:
        for (k, v) in node_util.iteritems():
            if v > 0:
                util = float(v / 8.0)
                f.write(",".join([str(k), str(util)]) + "\n")

def update_graph_capacity(sn, traffic_request, flow_id, mbox_spec, nmap_file,
        emap_file, seq_file, util_matrix, node_util, increase = True):
    sign = 1
    if not increase:
        sign = -1
    # update server capacity
    placement = []
    sequence = []
    with open(nmap_file, "r") as f:
        for line in reversed(f.readlines()):
            tokens = line.strip("\n\r").split(",")
            fid = int(tokens[0])
            if int(fid) == int(flow_id):
                placement = map(int, tokens[1:])
                break
    with open(seq_file, "r") as f:
        for line in reversed(f.readlines()):
            tokens = line.strip("\n\r").split(",")
            fid = int(tokens[0])
            if int(fid) == int(flow_id):
                sequence = map(int, tokens[1:])
                break

    if len(placement) > 0:
        # print placement
        # print sequence
        # print mbox_spec
        for i in range(0, len(placement)):
            idx = sequence[i]
            # print flow_id, idx
            ftype = int(traffic_request[int(flow_id)].mbox_seq[int(idx)])
            cpu_req = mbox_spec[ftype]
            server_id = placement[i]
            # print "Updating server " + str(server_id) + " with VNF " + str(idx) + " of type " + str(ftype) + " w. req. " + str(cpu_req)
            for node in sn.nodes(data = "all"):
                if node[0] == server_id:
                    node[1]['cpu'] += (sign * cpu_req)
                    break
            node_util[server_id] += (-sign * cpu_req)
    
    # Update internal switching capacity.
    if len(placement) > 0:
        for i in range(0, len(placement) - 1):
            u_theta, v_theta = placement[i], placement[i + 1]
            bw = traffic_request[int(flow_id)].bw
            if int(u_theta) == int(v_theta):
                for node in sn.nodes(data = "all"):
                    if node[0] == u_theta:
                        node[1]['bw'] += (sign * bw)
                        break

    # update link capacity capacity                
    with open(emap_file, "r") as f:
        for line in reversed(f.readlines()):
            tokens = line.strip("\n\r").split(",")
            fid = int(tokens[0])
            if int(fid) == int(flow_id):
                for k in range(1, len(tokens), 2):
                    u, v = int(tokens[k]), int(tokens[k + 1])
                    bw = traffic_request[int(flow_id)].bw
                    if u > v:
                        u, v = v, u
                    sn.get_edge_data(u, v)['bw'] += (sign * bw)
                    util_matrix[(u, v)] -= (sign * bw)
    return sn

def get_embedding_cost(cost_file, flow_id):
    ret = -1
    try:
        with open(cost_file) as f:
            for line in reversed(f.readlines()):
                tokens = line.strip("\n\r").split(",")
                if int(tokens[0]) == int(flow_id):
                    ret = int(float(tokens[1]))
                    break
    except IOError:
        pass
    return ret

def get_full_path(cur_directory, file_name):
    if not file_name.startswith("/"):
        return os.path.join(cur_directory, file_name)
    return file_name

def main():
    parser = argparse.ArgumentParser(
                formatter_class=argparse.ArgumentDefaultsHelpFormatter)
    parser.add_argument("--executable", type = str, required = True)
    parser.add_argument("--sn_topology_file", type = str, default = "univ2.topo")
    parser.add_argument("--flows_file", type = str, default = "traffic-request-khaleesi.dc")
    parser.add_argument("--mbox_spec_file", type = str, default = "mbox-spec-khaleesi")
    # parser.add_argument("--rcm_file", type = str, default = "mbox-rc-khaleesi")
    parser.add_argument("--log_prefix", type = str, default = "log.ilp.dc")
    parser.add_argument("--simulation_plan", type = str, 
                        default = "vnr-simulation")
    parser.add_argument("--max_simulation_time", type = int, default = 1000)
    args = parser.parse_args()
    
    traffic_requests = load_traffic_requests(args.flows_file)
    mbox_specs = load_mbox_specs(args.mbox_spec_file)

    current_time = 0
    event_queue = []
    current_directory = os.getcwd()

    with open(args.simulation_plan, "r") as f:
        for line in f:
            tokens = line.strip("\n\r").split(",")
            ts = int(tokens[0])
            end_time = int(tokens[1])
            # print end_time
            flow_id = tokens[2].rstrip("\n")
            e = Event(ts, "arrival", flow_id)
            # print e.debug_string()
            heapq.heappush(event_queue, e)
            if end_time <= args.max_simulation_time:
                e = Event(end_time, "departure", flow_id)
                # print e.debug_string()
                heapq.heappush(event_queue, e)
    sn = load_csv_graph(args.sn_topology_file)
    util_matrix = {}
    for u in sn.nodes():
        for v in sn.neighbors(u):
            if u < v:
                util_matrix[(u, v)] = 0.0
    node_util = {}
    for u in sn.nodes():
        node_util[u] = 0

    total_flows = 0
    accepted = 0
    rejected = 0
    while not len(event_queue) <= 0:
        e = heapq.heappop(event_queue)
        flow_id = e.flow_id
        print e.debug_string()
        # vn = load_csv_graph(args.vnr_directory + "/" + e.flow_id)
        if e.etype == "departure":
            # if the embedding of flow_id was not successful at the first place do
            # nothing. This can be checked by reading from $(flow_id).status file.
            # If there was a successful embedding increase graph's capacity.
            cost = get_embedding_cost(args.log_prefix + ".cost", flow_id)
            print "Cost = " + str(cost) + "\n"
            if cost <> -1:
                sn = update_graph_capacity(sn, traffic_requests, flow_id,
                        mbox_specs, args.log_prefix + ".nmap", args.log_prefix + ".emap",
                        args.log_prefix + ".sequence", util_matrix, node_util,
                        increase = True)
                write_csv_graph(sn, args.sn_topology_file)
                write_util_matrix(sn, util_matrix, "sim-data/util-data/util." + str(e.ts))
                write_server_util(sn, node_util, "sim-data/util-data/sutil." +  str(e.ts))
        elif e.etype == "arrival":
            # run embedding first. if embedding is successful decrease the
            # capacity of SN. Otherwise do nothing.
            total_flows += 1
            # sn_topology_file = get_full_path(current_directory, args.phys_topology)
            # flows_file = get_full_path(current_directory, 
            #                                  args.vnr_directory + "/" + e.flow_id)
            execute_one_experiment(args.executable, args.sn_topology_file,
                    args.flows_file, args.mbox_spec_file, args.log_prefix, 
                    e.flow_id)
            cost = get_embedding_cost(args.log_prefix + ".cost", flow_id)
            print "Cost: " + str(cost) + "\n"
            if cost <> -1:
                sn = update_graph_capacity(sn, traffic_requests, flow_id,
                        mbox_specs, args.log_prefix + ".nmap", args.log_prefix + ".emap",
                        args.log_prefix + ".sequence", util_matrix, node_util,
                        increase = False)
                write_csv_graph(sn, args.sn_topology_file)
                write_util_matrix(sn, util_matrix, "sim-data/util-data/util." + str(e.ts))
                write_server_util(sn, node_util, "sim-data/util-data/sutil." +  str(e.ts))
                accepted += 1
            with open("sim-data/sim-results", "a") as f:
                f.write(",".join([str(e.ts),str(total_flows), str(accepted)]) + "\n")
    print "total = " + str(total_flows) + ", accepted = " + str(accepted)
if __name__ == "__main__":
    main()
    

