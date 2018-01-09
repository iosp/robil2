/*
 * Filename: CostMapCell.cpp
 *   Author: Igor Makhtes
 *     Date: Jun 30, 2015
 *
 * The MIT License (MIT)
 *
 * Copyright (c) 2015
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */

#include <navex/costmap/CostMapCell.h>


/**
 * Unknown cell
 */
const CostMapCell::CellType CostMapCell::CELL_UNKNOWN = -1;

/**
 * Cell definitely free
 */
const CostMapCell::CellType CostMapCell::CELL_FREE = 0;

/**
 * Inflation radius, being in this cell may cause collision
 */
const CostMapCell::CellType CostMapCell::CELL_MAYBE_BLOCKED = 75;

/**
 * Being in guarantees collision
 */
const CostMapCell::CellType CostMapCell::CELL_BLOCKED = 100;

/**
 * Maximum value a cell can have
 */
const CostMapCell::CellType CostMapCell::CELL_MIN = -1;

/**
 * Maximum value a cell can have
 */
const CostMapCell::CellType CostMapCell::CELL_MAX = 100;
