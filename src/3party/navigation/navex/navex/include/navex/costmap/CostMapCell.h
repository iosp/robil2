/*
 * Filename: CostMapCell.h
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

#ifndef INCLUDE_NAVEX_COSTMAP_DATASOURCE_COSTMAPCELL_H_
#define INCLUDE_NAVEX_COSTMAP_DATASOURCE_COSTMAPCELL_H_


/**
 * CostMap cell's values enum
 */
class CostMapCell {

public:

	typedef signed char CellType;

	/**
	 * Unknown cell
	 */
	static const CellType CELL_UNKNOWN;

	/**
	 * Cell definitely free
	 */
	static const CellType CELL_FREE;

	/**
	 * Inflation radius, being in this cell may cause collision
	 */
	static const CellType CELL_MAYBE_BLOCKED;

	/**
	 * Being in guarantees collision
	 */
	static const CellType CELL_BLOCKED;

	/**
	 * Minimum value a cell can have
	 */
	static const CellType CELL_MIN;

	/**
	 * Maximum value a cell can have
	 */
	static const CellType CELL_MAX;

};


#endif /* INCLUDE_NAVEX_COSTMAP_DATASOURCE_COSTMAPCELL_H_ */
