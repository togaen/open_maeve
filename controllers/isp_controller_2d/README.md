# README #

This package defines a subsumption-based visual servoing controller that
computes Command2D messages given input guidance command and ISP fields. This
controller is designed to provide the local control component of a vision-based
[Selective Determinism](http://hdl.handle.net/2022/21670) architecture.

This controller is implemented as a library that is to be included in a node
that recieves and processes camera input into ISP fields. See the ar\_isp\_field
node for an example.

