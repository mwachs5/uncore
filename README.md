Uncore Library
==============

This is the repository for uncore components assosciated with Rocket chip
project. To uses these modules, include this repo as a git submodule within
the your chip repository and add it as Project in your chip's build.scala. 
These components are only dependent on Chisel, i.e.

    lazy val uncore = Project("uncore", file("uncore"), settings = buildSettings) dependsOn(chisel)

Documentation for the uncore library is available <a href="http://ucb-bar.github.io/uncore/latest/api/">here</a>
and an overview of the TileLink Protocol is available <a href="https://docs.google.com/document/d/1Iczcjigc-LUi8QmDPwnAu1kH4Rrt6Kqi1_EUaCrfrk8/pub">here</a>