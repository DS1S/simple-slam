package api

import (
	"net/http"

	"github.com/labstack/echo/v4"
)

type collectRequest struct {
	BoardID   boardID     `json:"board_id" validate:"required"`
	Spatials  [][]float32 `json:"spatials" validate:"required"`
	Positions [][]float32 `json:"positions" validate:"required"`
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
			make([][]float32, 0),
			make([][]float32, 0),
		}
	}

	points := boards[cr.BoardID]
	points.spatialPoints = append(points.spatialPoints, cr.Spatials...)
	points.positionPoints = append(points.positionPoints, cr.Positions...)

	c.Logger().Printf("Spatial Points: ", points.spatialPoints)
	return c.JSON(http.StatusOK, map[string]interface{}{
		"msg": "succesfully merged point data",
	})
}
