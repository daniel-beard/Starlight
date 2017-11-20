//
//  Starlight.swift
//  Starlight
//
//  Created by Daniel Beard on 9/13/15.
//  Copyright © 2015 DanielBeard. All rights reserved.
//

import Foundation

typealias StateLinkedList = Array<State>
extension Array {
    mutating func addFirst(_ element: Element) {
        if self.count == 0 {
            append(element)
        } else {
            insert(element, at: 0)
        }
    }
    
    public var description: String {
        get {
            var result = ""
            for item in self {
                result += "\(item)\n"
            }
            return result
        }
    }
}

class PriorityQueue<T: Comparable> {
    var heap = [T]()
    
    func push(item: T) {
        heap.append(item)
        heap.sort()
    }
    
    func pop() -> T {
        return heap.removeFirst()
    }
    
    func peek() -> T {
        return (heap.first)!
    }
    
    func isEmpty() -> Bool {
        return count == 0
    }
    
    var count: Int {
        get {
            return heap.count
        }
    }
}

struct CellInfo {
    var g = 0.0
    var rhs = 0.0
    var cost = 0.0
}

public struct Point {
    var x = 0
    var y = 0
}

public class Starlight {
    
    //MARK: Private Properties
    private var path = [State]()
    
    private var k_m = 0.0
    private var s_start = State()
    private var s_goal = State()
    private var s_last = State()
    private var openList = PriorityQueue<State>()
    private var cellHash = [State: CellInfo]()
    private var openHash = [State: Double]()
    
    //MARK: Constants
    private let maxSteps = 80000
    private var C1 = 1.0
    
    init(start: Point, goal: Point) {
        s_start.x = start.x
        s_start.y = start.y
        s_goal.x = goal.x
        s_goal.y = goal.y
        
        let goalCellInfo = CellInfo(g: 0.0, rhs: 0.0, cost: C1)
        cellHash[s_goal] = goalCellInfo
        
        let startHeuristic = heuristic(s_start, b: s_goal)
        let startCellInfo = CellInfo(g: startHeuristic, rhs: startHeuristic, cost: C1)
        cellHash[s_start] = startCellInfo
        
        s_start = calculateKey(s_start)
        s_last = s_start
    }
    
    //MARK: Private Methods
    
    /// CalculateKey - > As per [S. Koenig, 2002]
    private func calculateKey(_ u: State) -> State {
        let val = min(getRHS(u), getG(u))
        let first = (val + heuristic(u, b: s_start) + k_m)
        let second = val
        return State(x: u.x, y: u.y, k: Pair<Double>(first, second))
    }
    
    /// Returns the rhs value for the state u
    private func getRHS(_ u: State) -> Double {
        guard u != s_goal else {
            return 0.0
        }
        if cellHash[u] == nil {
            return heuristic(u, b: s_goal)
        }
        return cellHash[u]!.rhs
    }

    /// As per [S. Koenig,2002] except for two main modifications:
    /// 1. We stop planning after a number of steps, 'maxsteps' we do this
    ///    because this algorithm can plan forever if the start is surrounded  by obstacles
    /// 2. We lazily remove states from the open list so we never have to iterate through it.
    private func computeShortestPath() -> Int {
        var s = StateLinkedList()
        
        if openList.isEmpty() { return 1 }
        
        var k = 0
        while !openList.isEmpty() {

            // Update start
            s_start = calculateKey(s_start)

            // Bail if our conditions aren't met
            guard (openList.peek() < s_start || getRHS(s_start) != getG(s_start)) else {
                break
            }

            k += 1
            if k > maxSteps {
                print("At maxsteps")
                return -1
            }

            var u = State()
            let test = getRHS(s_start) != getG(s_start)

            // Lazy remove
            while (true) {
                if openList.isEmpty() { return 1 }
                u = openList.pop()
                if !isValid(u) { continue }
                if !(u < s_start) && !test { return 2 }
                break
            }

            openHash[u] = nil
            let k_old = State(state: u)
            u = calculateKey(u)

            if k_old < u { // u is out of date
                insert(u)
            } else if getG(u) > getRHS(u) { // needs update (got better)
                setG(u, g: getRHS(u))
                s = getPred(u)
                for state in s {
                    updateVertex(state)
                }
            } else { // g <= rhs, state has got worse
                setG(u, g: Double.infinity)
                s = getPred(u)

                for state in s {
                    updateVertex(state)
                }
                updateVertex(u)
            }
            s_start = calculateKey(s_start)
        }
        
        return 0
    }

    /// Returns a list of successor states for state u, since this is an
    /// 8-way graph this list contains all of a cells neighbours. Unless
    /// the cell is occupied, in which case it has no successors.
    private func getSucc(_ u: State) -> StateLinkedList {
        var s = StateLinkedList()
        var tempState = State()
        if occupied(u) { return s }
        
        // Generating the successors, starting at the immediate right,
        // moving in a clockwise manner
        tempState = State(x: u.x + 1, y: u.y, k: Pair(-1.0,-1.0))
        s.addFirst(tempState)
        tempState = State(x: u.x + 1, y: u.y + 1, k: Pair(-1.0,-1.0))
        s.addFirst(tempState)
        tempState = State(x: u.x, y: u.y + 1, k: Pair(-1.0,-1.0))
        s.addFirst(tempState)
        tempState = State(x: u.x - 1, y: u.y + 1, k: Pair(-1.0,-1.0))
        s.addFirst(tempState)
        tempState = State(x: u.x - 1, y: u.y, k: Pair(-1.0,-1.0))
        s.addFirst(tempState)
        tempState = State(x: u.x - 1, y: u.y - 1, k: Pair(-1.0,-1.0))
        s.addFirst(tempState)
        tempState = State(x: u.x, y: u.y - 1, k: Pair(-1.0,-1.0))
        s.addFirst(tempState)
        tempState = State(x: u.x + 1, y: u.y - 1, k: Pair(-1.0,-1.0))
        s.addFirst(tempState)
        return s
    }
    
    /// Returns a list of all the predecessor states for state u. Since
    /// this is for an 8-way connected graph, the list contains all the
    /// neighbours for state u. Occupied neighbours are not added to the list
    private func getPred(_ u: State) -> StateLinkedList {
        var s = StateLinkedList()
        var tempState = State()
        
        tempState = State(x: u.x + 1, y: u.y, k: Pair(-1.0,-1.0))
        if !occupied(tempState) { s.addFirst(tempState) }
        tempState = State(x: u.x + 1, y: u.y + 1, k: Pair(-1.0,-1.0))
        if !occupied(tempState) { s.addFirst(tempState) }
        tempState = State(x: u.x, y: u.y + 1, k: Pair(-1.0,-1.0))
        if !occupied(tempState) { s.addFirst(tempState) }
        tempState = State(x: u.x - 1, y: u.y + 1, k: Pair(-1.0,-1.0))
        if !occupied(tempState) { s.addFirst(tempState) }
        tempState = State(x: u.x - 1, y: u.y, k: Pair(-1.0,-1.0))
        if !occupied(tempState) { s.addFirst(tempState) }
        tempState = State(x: u.x - 1, y: u.y - 1, k: Pair(-1.0,-1.0))
        if !occupied(tempState) { s.addFirst(tempState) }
        tempState = State(x: u.x, y: u.y - 1, k: Pair(-1.0,-1.0))
        if !occupied(tempState) { s.addFirst(tempState) }
        tempState = State(x: u.x + 1, y: u.y - 1, k: Pair(-1.0,-1.0))
        if !occupied(tempState) { s.addFirst(tempState) }
        return s;
    }
    
    /// As per [S. Koenig, 2002]
    private func updateVertex(_ u: State) {
        var states = StateLinkedList()
        if u != s_goal {
            states = getSucc(u)
            var tmp = Double.infinity
            var tmp2 = 0.0
            for state in states {
                tmp2 = getG(state) + cost(u, b: state)
                if tmp2 < tmp { tmp = tmp2 }
            }
            if !close(getRHS(u), y: tmp) { setRHS(u, rhs: tmp) }
        }
        if !close(getG(u), y: getRHS(u)) { insert(u) }
    }
    
    /// Returns true if state u is on the open list or not by checking if it is in the hash table.
    private func isValid(_ u: State) -> Bool {
        if openHash[u] == nil { return false }
        if !close(keyHashCode(u), y: openHash[u]!) { return false }
        return true
    }
    
    /// Returns the value for the state u
    private func getG(_ u: State) -> Double {
        if cellHash[u] == nil {
            return heuristic(u, b: s_goal)
        }
        return cellHash[u]!.g
    }
    
    /// The heuristic we use is the 8-way distance
    /// scaled by a constant C1 (should be set to <= min cost
    private func heuristic(_ a: State, b: State) -> Double {
        return eightCondist(a, b: b) * C1
    }
    
    /// Returns the 8-way distance between state a and state b
    private func eightCondist(_ a: State, b: State) -> Double {
        var min = Double(abs(a.x - b.x))
        var max = Double(abs(a.y - b.y))
        if min > max {
            swap(&min, &max)
        }
        return (2.squareRoot() - 1.0) * min + max
    }
    
    /// Inserts state into openList and openHash
    private func insert(_ u: State) {
        let u = calculateKey(u)
        let csum = keyHashCode(u)
        openHash[u] = csum
        openList.push(item: u)
    }
    
    /// Returns the key hash code for the state u, this is used to compare
    /// a state that has been updated
    private func keyHashCode(_ u: State) -> Double {
        return Double(u.k.first + 1193 * u.k.second)
    }
    
    /// Returns true if the cell is occupied (non-traversable), false
    /// otherwise. Non-traversable are marked with a cost < 0
    private func occupied(_ u: State) -> Bool {
        if let cell = cellHash[u] {
            return (cell.cost < 0)
        } else {
            return false
        }
    }
    
    /// Euclidean cost between state a and state b
    private func trueDist(_ a: State, b: State) -> Double {
        let x = Double(a.x - b.x)
        let y = Double(a.y - b.y)
        return sqrt(x * x + y * y)
    }
    
    /** Returns the cost of moving from state a to state b. This could be
    either the cost of moving off state a or onto state b, we went with the
    former. This is also the 8-way cost. */
    private func cost(_ a: State, b: State) -> Double {
        let xd = abs(a.x - b.x)
        let yd = abs(a.y - b.y)
        var scale = 1.0
        if xd + yd > 1 { scale = 2.squareRoot() }
        if cellHash.keys.contains(a) == false { return scale * C1 }
        return scale * cellHash[a]!.cost
    }
    
    /// Returns true if x and y are within 10E-5, false otherwise
    private func close(_ x: Double, y: Double) -> Bool {
        if x == Double.infinity && y == Double.infinity { return true }
        return abs(x - y) < 0.00001
    }
    
    /// Sets the G value for state u
    private func setG(_ u: State, g: Double) {
        makeNewCell(u)
        cellHash[u]!.g = g
    }
    
    /// Sets the rhs value for state u
    private func setRHS(_ u: State, rhs: Double) {
        makeNewCell(u)
        cellHash[u]!.rhs = rhs
    }
    
    /// Checks if a cell is in the hash table, if not it adds it in.
    private func makeNewCell(_ u: State) {
        guard cellHash[u] == nil else { return }
        let heuristicValue = heuristic(u, b: s_goal)
        let cellInfo = CellInfo(g: heuristicValue, rhs: heuristicValue, cost: C1)
        cellHash[u] = cellInfo
    }
    
    //MARK: Public Methods
    
    public func replan() -> Bool {
        path.removeAll()
        
        let res = computeShortestPath()
        if res < 0 {
            print("No path to goal")
            return false
        }
        
        var n = StateLinkedList()
        var cur = s_start
        
        if getG(s_start) == Double.infinity {
            print("No path to goal")
            return false
        }
       
        while (cur != s_goal) {
            path.append(cur)
            n = StateLinkedList()
            n = getSucc(cur)
            
            if n.isEmpty {
                print("No path to goal")
                return false
            }
            
            var cmin = Double.infinity
            var tmin = 0.0
            var smin = State()
            
            for state in n {
                var val = cost(cur, b: state)
                let val2 = trueDist(state, b: s_goal) + trueDist(s_start, b: state)
                val += getG(state)
                
                if close(val, y: cmin) {
                    if tmin > val2 {
                        tmin = val2
                        cmin = val
                        smin = state
                    }
                } else if val < cmin {
                    tmin = val2
                    cmin = val
                    smin = state
                }
            }
            n.removeAll()
            cur = State(state: smin)
        }
        path.append(s_goal)
        return true
    }
    
    /// Update the position of the agent/robot.
    /// This does not force a replan.
    public func updateStart(x: Int, y: Int) {
        s_start.x = x
        s_start.y = y
        k_m += heuristic(s_last, b: s_start)
        
        s_start = calculateKey(s_start)
        s_last = s_start
    }
    
    public func updateGoal(x: Int, y: Int) {
        //TODO: Implement this
        fatalError("Not implemented")
    }
    
    /// updateCell as per [S. Koenig, 2002]
    public func updateCell(x: Int, y: Int, value: Double) {
        var u = State()
        u.x = x; u.y = y
        if u == s_start || u == s_goal {
            return
        }
        
        makeNewCell(u)
        cellHash[u]!.cost = value
        updateVertex(u)
    }
    
    public func getPath() -> [State] {
        return self.path
    }
}

struct Pair<T> {
    var first: T
    var second: T
    
    init(_ f: T, _ s: T) {
        first = f
        second = s
    }
}

public struct State: Hashable, Comparable, CustomStringConvertible {
    var x = 0
    var y = 0
    var k: Pair<Double> = Pair(0.0, 0.0)
    
    init() { }
    init(x: Int, y: Int, k: Pair<Double>) {
        self.x = x
        self.y = y
        self.k = k
    }
    
    init(state: State) {
        x = state.x
        y = state.y
        k = state.k
    }
    
    public var hashValue: Int {
        get { return x + 34245 * y }
    }
    
    public var description: String {
        get { return "x: \(x) y: \(y)\n" }
    }
}

public func ==(lhs: State, rhs: State) -> Bool {
    return lhs.x == rhs.x && lhs.y == rhs.y
}

public func <(lhs: State, rhs: State) -> Bool {
    let delta = 0.000001
    if lhs.k.first + delta < rhs.k.first {
        return true
    } else if lhs.k.first - delta > rhs.k.first {
        return false
    }
    return lhs.k.second < rhs.k.second
}
