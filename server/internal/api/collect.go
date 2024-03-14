package api

import (
	"net/http"

	"github.com/labstack/echo/v4"
)

type collectRequest struct {
	BoardID   boardID `json:"board_id" validate:"required"`
	Spatials  [][]int `json:"spatials" validate:"required"`
	Positions [][]int `json:"positions" validate:"required"`
}

func collect(c echo.Context) error {
	var cr collectRequest

	err := c.Bind(&cr)
	if err != nil {
		c.Logger().Errorf("Invalid collect request body: %s", err)
		return echo.NewHTTPError(http.StatusBadRequest, "Invalid request body")
	}

	err = c.Validate(&cr)
	if err != nil {
		return err
	}

	_, ok := boards[cr.BoardID]
	if !ok {
		boards[cr.BoardID] = &positionData{
			make([][]int, 0),
			make([][]int, 0),
		}
	}

	points := boards[cr.BoardID]
	points.spatialPoints = append(points.spatialPoints, cr.Spatials...)
	points.positionPoints = append(points.positionPoints, cr.Positions...)

	c.Logger().Printf("Spatial Points: ", points.spatialPoints)
	return c.JSON(http.StatusOK, map[string]interface{}{
		"msg": points.spatialPoints,
	})
}
