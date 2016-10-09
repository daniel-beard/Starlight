# Starlight ![](https://img.shields.io/badge/swift-3-brightgreen.svg) [![Build Status](https://travis-ci.org/daniel-beard/Starlight.svg?branch=master)](https://travis-ci.org/daniel-beard/Starlight)
A swift implementation of D*Lite (DStarLite) a pathfinding algorithm

### Usage

```
let pf = Starlight(start: Point(x: 0, y: 1), goal: Point(x: 3, y: 1))
pf.updateCell(x: 2, y: 1, value: -1)
pf.updateCell(x: 2, y: 0, value: -1)
pf.updateCell(x: 2, y: 2, value: -1)
pf.updateCell(x: 3, y: 0, value: -1)
_ = pf.replan()

let path = pf.getPath()
print("PATH: \(path)")
```

Prints:

```
PATH: [x: 0 y: 1
, x: 1 y: 2
, x: 2 y: 3
, x: 3 y: 2
, x: 3 y: 1
]
```

### License 
MIT
