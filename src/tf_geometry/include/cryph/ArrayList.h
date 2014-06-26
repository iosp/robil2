// ArrayList.h
// This is OPEN SOURCE software developed by James R. Miller (jrmiller@ku.edu)
// Original version: ca. 1996. See README_*.txt for more information.

#ifndef ARRAYLIST_H
#define ARRAYLIST_H

#include <stdlib.h>

template <typename T>
class ArrayList
{
public:
	ArrayList(int initSize=10) : nItems(0), listLength(initSize)
	{
		if (listLength <= 0)
			listLength = 10;
		array = new T[listLength];
	}
	virtual ~ArrayList()
	{
		if (array != NULL)
			delete [] array;
		array = NULL;
		nItems = listLength = 0;
	}
	void add(T item)
	{
		if (nItems == listLength)
			ensureCapacity(2*listLength);
		array[nItems++] = item;
	}
	void clear()
	{
		nItems = 0;
	}
	void ensureCapacity(int minCapacity)
	{
		if (minCapacity > listLength)
		{
			listLength = minCapacity;
			T* newArray = new T[listLength];
			for (int i=0 ; i<nItems ; i++)
				newArray[i] = array[i];
			delete [] array;
			array = newArray;
		}
	}
	const T* getArray()
	{
		return array;
	}
	T get(int index) const
	{
		if ((index < 0) || (index >= listLength))
			return array[0]; // throw IndexOutOfBoundsException();
		return array[index];
	}
	bool isEmpty() const
	{
		return nItems == 0;
	}
	void remove(int index)
	{
		if ((index < 0) || (index >= listLength))
			return; // throw IndexOutOfBoundsException();
		nItems--;
		for (int i=index ; i<nItems ; i++)
			array[i] = array[i+1];
	}
	void set(int index, T item)
	{
		if ((index < 0) || (index >= listLength))
			return; // throw IndexOutOfBoundsException();
		array[index] = item;
	}
	int size() const
	{
		return nItems;
	}
private:
	T* array;
	int nItems, listLength;
};

#endif
