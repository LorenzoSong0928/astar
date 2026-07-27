## 1 网格

- 是二维数组，目前初始化为十行十列的全0（根据后续需要可以更改）
  
  - 二维网格中x,y是grid[y][x]

- 为了展示网格分布，采用分行打印
  
  - 同时采用 `print(*row)`，去掉列表显示中的逗号和中括号。

- x,y = (2,3)
  
  - 是python的序列解包
  
  - 将序列中的元素依次赋值给多个变量
    
    - 也可以x,y = [2,3]
  
  - 要求**变量与元素数量一致**

## 2 `set` 集合

- 创建空集合：`s = set()`，`{}` 是空字典。

- 添加元素：`s.add(x)`；删除元素：`s.remove(x)`。

- 元素不重复、没有固定顺序。

- 元素必须可哈希，如数字、字符串、元组；不能直接放 `list`、`dict`、`set`。

- 常用于快速判断：`x in s`，平均时间复杂度约为 $O(1)$。

```python
visited = set()
visited.add((0, 0))

if (0, 0) in visited:
    print("访问过")
```

## 3 `heapq` 最小堆

- `heapq` 用列表实现优先队列，堆顶始终是最小元素。

- 入堆：`heapq.heappush(heap, value)`

- 弹出最小值：`heapq.heappop(heap)`

- 查看最小值但不删除：`heap[0]`

- A* 中通常存：`(优先级, 节点)`。

```python
import heapq

open_set = []
heapq.heappush(open_set, (0, start))

priority, current = heapq.heappop(open_set)
```

注意：优先级相同时，Python 会继续比较元组后面的元素；自定义节点不可比较时，可以加入计数器：

```python
heapq.heappush(open_set, (priority, counter, node))
```

## 4 Dijkstra 最短路径搜索

Dijkstra 用于寻找起点到其他节点的最小累计代价。

在当前网格中，每移动一格的代价设为 `1`。

### 核心变量

```python
open_set = []           # 等待处理的节点
cost = {start: 0}       # 起点到各节点的最小代价
parent = {start: None}  # 记录节点从哪里到达
visited = set()         # 已经处理完成的节点
```

- `open_set`
  
  - 使用 `heapq` 实现优先队列。
  
  - 每次取出当前累计代价最小的节点。

- `cost`
  
  - `cost[node]` 表示从起点到 `node` 的当前最小代价。

- `parent`
  
  - 用于记录路径。
  
  - 写法为：

```python
parent[子节点] = 父节点
```

- `visited`
  
  - 保存已经确定最短代价的节点。
  
  - 防止节点被重复处理。

### 核心更新过程

```python
for neighbour in get_neighbours(current, grid):
    step_cost = 1
    new_cost = current_cost + step_cost

    if neighbour not in cost or new_cost < cost[neighbour]:
        cost[neighbour] = new_cost
        parent[neighbour] = current
        heapq.heappush(open_set, (new_cost, neighbour))
```

执行逻辑：

```text
获取合法邻居
→ 计算到达邻居的新代价
→ 判断是否第一次到达或找到更短路线
→ 更新最小代价
→ 记录父节点
→ 将邻居加入优先队列
```

### 完整搜索框架

```python
def dijkstra(grid, start, goal):
    open_set = []
    heapq.heappush(open_set, (0, start))

    cost = {start: 0}
    parent = {start: None}
    visited = set()

    while open_set:
        current_cost, current = heapq.heappop(open_set)

        if current in visited:
            continue

        visited.add(current)

        if current == goal:
            break

        for neighbour in get_neighbours(current, grid):
            step_cost = 1
            new_cost = current_cost + step_cost

            if neighbour not in cost or new_cost < cost[neighbour]:
                cost[neighbour] = new_cost
                parent[neighbour] = current
                heapq.heappush(open_set, (new_cost, neighbour))

    return parent, cost
```

### 编写时遇到的问题

#### 1. 把列表当作函数调用

错误写法：

```python
step_cost = grid(neighbour)
```

`grid` 是列表，圆括号 `()` 表示调用函数，因此不能这样访问。

列表访问应使用方括号：

```python
grid[y][x]
```

但当前网格中的 `0` 和 `1` 表示通行状态，不是移动代价，因此应单独设置：

```python
step_cost = 1
```

记忆：

```text
函数调用使用 ()
列表访问使用 []
网格状态和移动代价是两类信息
```

#### 2. 直接访问不存在的字典键

错误写法：

```python
if new_cost < cost[neighbour]:
```

第一次发现 `neighbour` 时，它还不在 `cost` 字典中，会产生 `KeyError`。

正确写法：

```python
if neighbour not in cost or new_cost < cost[neighbour]:
```

含义：

- 第一次发现该节点时，记录它；

- 已经发现过，但新路线更短时，更新它。

#### 3. 父节点方向写反

错误写法：

```python
parent[current] = neighbour
```

搜索方向是：

```text
current → neighbour
```

因此正确记录方式是：

```python
parent[neighbour] = current
```

记忆：

```text
parent[子节点] = 父节点
```

之后才能从终点沿着父节点不断返回起点。

#### 4. 只有有效更新才加入堆

只有第一次发现节点或找到更短路径时，才需要重新压入优先队列：

```python
if neighbour not in cost or new_cost < cost[neighbour]:
    cost[neighbour] = new_cost
    parent[neighbour] = current
    heapq.heappush(open_set, (new_cost, neighbour))
```

否则会产生不必要的重复节点。

### 本阶段记忆点

```text
open_set：下一步可能处理谁
visited：谁已经处理完成
cost：到达节点的最小累计代价
parent：最短路径从哪里走来
```

```text
parent[子节点] = 父节点
```

```text
只有第一次发现节点或找到更短路线时，才更新并加入堆。
```
