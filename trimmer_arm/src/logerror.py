#!/usr/bin/env python -OO
# encoding: utf-8

"""
logerror.py - Contains a set of functions that facilitate special error 
logs that account for control code and Hardware Module special cases

Created by William Woodall on 2010-02-22.
"""
__author__ = "William Woodall"
__copyright__ = "Copyright (c) 2010 John Harrison, William Woodall"

###  Imports  ###

# Standard Python Libraries
import sys
import os
import traceback
import linecache

###  Class  ###


###  Functions  ###

def logError(exc_info, log_func, msg, line_no_delta=0):
    """Logs an error with a traceback"""
    exceptionType, exceptionValue, exceptionTraceback = exc_info
    tb_list = format_exception(exceptionType, exceptionValue, exceptionTraceback, special_file_offset=line_no_delta)
    tb_message = ''.join(tb_list)
    tb_message = tb_message[:-1]
    message = msg+'\n'+tb_message
    log_func(message)
    
def format_exception(etype, value, tb, limit = None, special_file_offset=0):
    """Format a stack trace and the exception information.

    The arguments have the same meaning as the corresponding arguments
    to print_exception().  The return value is a list of strings, each
    ending in a newline and some containing internal newlines.  When
    these lines are concatenated and printed, exactly the same text is
    printed as does print_exception().
    """
    if tb:
        list = ['Traceback (most recent call last):\n']
        list = list + format_tb(tb, limit, special_file_offset)
    else:
        list = []
    list = list + traceback.format_exception_only(etype, value)
    return list

def format_tb(tb, limit = None, special_file_offset=0):
    """A shorthand for 'format_list(extract_stack(f, limit))."""
    return traceback.format_list(extract_tb(tb, limit, special_file_offset))

def extract_tb(tb, limit = None, special_line_offset=0):
    """Copied from tracebac.py and modified to accomodate cc and hwm"""
    if limit is None:
        if hasattr(sys, 'tracebacklimit'):
            limit = sys.tracebacklimit
    list = []
    n = 0
    while tb is not None and (limit is None or n < limit):
        f = tb.tb_frame
        lineno = tb.tb_lineno
        co = f.f_code
        filename = co.co_filename
        # If .cc or .hwm correct the line num
        if filename[-3:] == '.cc' or filename[-4:] == '.hwm':
            lineno -= special_line_offset
        name = co.co_name
        linecache.checkcache(filename)
        line = linecache.getline(filename, lineno, f.f_globals)
        if line: line = line.strip()
        else: line = None
        list.append((filename, lineno, name, line))
        tb = tb.tb_next
        n = n+1
    return list
