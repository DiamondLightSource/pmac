.. _introduction:

Introduction
============

The pmac module has been implemented as a replacement for the existing tpmac, pmacUtil and pmacCoord modules.  It contains updated type III asyn driver classes for both standard motors and for coordinate system axes.  It also implements trajectory scanning for PMAC motion controllers.  The communications method has been updated to improve the efficiency of status requests, by batching groups of status items into a single request message.
 
