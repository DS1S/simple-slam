package api

import "github.com/labstack/echo/v4"

type boardID string

type positionData struct {
	spatialPoints  [][]float32
	positionPoints [][]float32
}

var boards map[boardID]*positionData

func WithRoutes(e *echo.Echo) {
	g := e.Group("api")
	g.POST("/collect", collect)
	g.DELETE("/reset/:board_id", reset)
	g.GET("/points/:board_id", points)
}

func init() {
	boards = make(map[boardID]*positionData)
}
