'''
!/usr/bin/env python3

sorted_queue.py
A sorted queue.
'''

class SortedQueue():
    def __init__(self, comparator):
        '''
        Initialize queue and variable.
        '''

        self.queue = []
        self.comparator = comparator
    
    def push(self, value):
        '''
        Push value into queue and sorts it.

        :param value: value to push.    
        '''

        self.queue.append(value)
        self.queue = sorted(self.queue, key=self.comparator)
    
    def pop(self):
        '''
        Pops first element queue and return.

        :return: first element of the queue
        '''
        return self.queue.pop(0)
    
    def remove(self, value):
        '''
        Removes a specific value from the queue.

        :return: the removed value, None if not found in queue
        '''

        if(value in self.queue):
            return self.queue.remove(value)
        return None

    def size(self):
        '''
        Gets the size of the queue.

        :return: size of the queue
        '''

        return len(self.queue)