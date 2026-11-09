#include <stdio.h>
#include <string.h>
#include <limits.h>
#include "resq_c.h"

int resq_graph_find_location(const ResqGraph *graph, const char *name) {
    if (graph == NULL || name == NULL) return -1;
    for (int i = 0; i < graph->count; ++i) {
        if (strcmp(graph->names[i], name) == 0) return i;
    }
    return -1;
}

int resq_graph_add_location(ResqGraph *graph, const char *name) {
    if (graph == NULL || name == NULL || graph->count >= RESQ_MAX_NODES) return -1;
    if (resq_graph_find_location(graph, name) >= 0) return -1;
    int index = graph->count++;
    snprintf(graph->names[index], RESQ_MAX_LOCATION, "%s", name);
    for (int i = 0; i < RESQ_MAX_NODES; ++i) {
        graph->weights[index][i] = 0;
        graph->weights[i][index] = 0;
        graph->blocked[index][i] = 0;
        graph->blocked[i][index] = 0;
    }
    return index;
}

int resq_graph_add_road(ResqGraph *graph, int from, int to, int weight) {
    if (graph == NULL || from < 0 || to < 0 || from >= graph->count || to >= graph->count) return 0;
    if (from == to || weight <= 0) return 0;
    graph->weights[from][to] = weight;
    graph->weights[to][from] = weight;
    graph->blocked[from][to] = 0;
    graph->blocked[to][from] = 0;
    return 1;
}

int resq_graph_block_road(ResqGraph *graph, int from, int to, int blocked) {
    if (graph == NULL || from < 0 || to < 0 || from >= graph->count || to >= graph->count) return 0;
    if (graph->weights[from][to] == 0) return 0;
    graph->blocked[from][to] = blocked ? 1 : 0;
    graph->blocked[to][from] = blocked ? 1 : 0;
    return 1;
}

void resq_graph_print(const ResqGraph *graph) {
    if (graph == NULL || graph->count == 0) {
        printf("No locations in the map.\n");
        return;
    }
    printf("\nRoad map\n");
    for (int i = 0; i < graph->count; ++i) {
        printf("%d. %s\n", i + 1, graph->names[i]);
        for (int j = i + 1; j < graph->count; ++j) {
            if (graph->weights[i][j] > 0) {
                printf("   -> %s | cost %d | %s\n", graph->names[j], graph->weights[i][j],
                       graph->blocked[i][j] ? "BLOCKED" : "OPEN");
            }
        }
    }
}

void resq_graph_bfs(const ResqGraph *graph, int start) {
    int visited[RESQ_MAX_NODES] = {0};
    int queue[RESQ_MAX_NODES];
    int front = 0, rear = 0;
    if (graph == NULL || start < 0 || start >= graph->count) {
        printf("Invalid starting location.\n");
        return;
    }
    visited[start] = 1;
    queue[rear++] = start;
    printf("Reachable locations from %s: ", graph->names[start]);
    while (front < rear) {
        int current = queue[front++];
        printf("%s ", graph->names[current]);
        for (int next = 0; next < graph->count; ++next) {
            if (graph->weights[current][next] > 0 &&
                !graph->blocked[current][next] && !visited[next]) {
                visited[next] = 1;
                queue[rear++] = next;
            }
        }
    }
    printf("\n");
}

int resq_graph_dijkstra(const ResqGraph *graph, int start, int target, int *path, int *path_len) {
    int dist[RESQ_MAX_NODES];
    int previous[RESQ_MAX_NODES];
    int visited[RESQ_MAX_NODES] = {0};
    if (path_len != NULL) *path_len = 0;
    if (graph == NULL || path == NULL || path_len == NULL ||
        start < 0 || target < 0 || start >= graph->count || target >= graph->count) return -1;
    for (int i = 0; i < graph->count; ++i) {
        dist[i] = INT_MAX;
        previous[i] = -1;
    }
    dist[start] = 0;
    for (int step = 0; step < graph->count; ++step) {
        int current = -1;
        int best = INT_MAX;
        for (int i = 0; i < graph->count; ++i) {
            if (!visited[i] && dist[i] < best) {
                best = dist[i];
                current = i;
            }
        }
        if (current == -1) break;
        visited[current] = 1;
        if (current == target) break;
        for (int next = 0; next < graph->count; ++next) {
            int weight = graph->weights[current][next];
            if (weight <= 0 || graph->blocked[current][next] || visited[next]) continue;
            if (dist[current] <= INT_MAX - weight && dist[current] + weight < dist[next]) {
                dist[next] = dist[current] + weight;
                previous[next] = current;
            }
        }
    }
    if (dist[target] == INT_MAX) return -1;
    int reverse[RESQ_MAX_NODES];
    int count = 0;
    for (int node = target; node != -1; node = previous[node]) {
        reverse[count++] = node;
        if (node == start) break;
    }
    if (reverse[count - 1] != start) return -1;
    for (int i = 0; i < count; ++i) path[i] = reverse[count - i - 1];
    *path_len = count;
    return dist[target];
}
