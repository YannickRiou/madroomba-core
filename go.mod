module github.com/YannickRiou/madroomba-core

go 1.18

require (
	github.com/YannickRiou/roomba-go-lib v0.0.0-00010101000000-000000000000
	github.com/tarm/serial v0.0.0-20180830185346-98f6abe2eb07
)

require golang.org/x/sys v0.17.0 // indirect

replace github.com/YannickRiou/roomba-go-lib => ../roomba-go-lib
