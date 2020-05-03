def min_weighted_dominating_set(G, weight=None):
    r"""Return minimum weight vertex dominating set.

    Parameters
    ----------
    G : NetworkX graph
      Undirected graph

    weight : None or string, optional (default = None)
        If None, every edge has weight/distance/weight 1. If a string, use this
        edge attribute as the edge weight. Any edge attribute not present
        defaults to 1.

    Returns
    -------
    min_weight_dominating_set : set
      Returns a set of vertices whose weight sum is no more than log w(V) * OPT

    Notes
    -----
    This algorithm computes an approximate minimum weighted dominating set
    for the graph G. The upper-bound on the size of the solution is
    log w(V) * OPT.  Runtime of the algorithm is `O(|E|)`.

    References
    ----------
    .. [1] Vazirani, Vijay Approximation Algorithms (2001)
    """
    if not G:
        raise ValueError("Expected non-empty NetworkX graph!")

    # min cover = min dominating set
    dom_set = set([])
    cost_func = dict((node, nd.get(weight, 1)) \
                     for node, nd in G.nodes_iter(data=True))

    vertices = set(G)
    sets = dict((node, set([node]) | set(G[node])) for node in G)

    def _cost(subset):
        """ Our cost effectiveness function for sets given its weight
        """
        cost = sum(cost_func[node] for node in subset)
        return cost / float(len(subset - dom_set))

    while vertices:
        # find the most cost effective set, and the vertex that for that set
        dom_node, min_set = min(sets.items(),
                                key=lambda x: (x[0], _cost(x[1])))
        alpha = _cost(min_set)

        # reduce the cost for the rest
        for node in min_set - dom_set:
            cost_func[node] = alpha

        # add the node to the dominating set and reduce what we must cover
        dom_set.add(dom_node)
        del sets[dom_node]
        vertices = vertices - min_set

    return dom_set