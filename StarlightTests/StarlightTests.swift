//
//  StarlightTests.swift
//  StarlightTests
//
//  Created by Daniel Beard on 10/9/16.
//  Copyright © 2016 DanielBeard. All rights reserved.
//

import XCTest
@testable import Starlight

class StarlightTests: XCTestCase {
    
    func testExample() {
        let pf = Starlight(start: Point(x: 0, y: 1), goal: Point(x: 3, y: 1))
        pf.updateCell(x: 2, y: 1, value: -1)
        pf.updateCell(x: 2, y: 0, value: -1)
        pf.updateCell(x: 2, y: 2, value: -1)
        pf.updateCell(x: 3, y: 0, value: -1)
        _ = pf.replan()

        let path = pf.getPath()

        XCTAssert(path.count == 5)
        let pair = Pair<Double>(1, 1)
        XCTAssert(path[0] == State(x: 0, y: 1, k: pair))
        XCTAssert(path[1] == State(x: 1, y: 2, k: pair))
        XCTAssert(path[2] == State(x: 2, y: 3, k: pair))
        XCTAssert(path[3] == State(x: 3, y: 2, k: pair))
        XCTAssert(path[4] == State(x: 3, y: 1, k: pair))
        
        print("PATH: \(path)")
    }
}
