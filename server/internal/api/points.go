package api

import (
	"net/http"

	"github.com/labstack/echo/v4"
)

type pointsRequest struct {
	BoardID boardID `param:"board_id" validate:"required"`
	Mock    bool    `query:"mock" default:"false"`
}

type pointsResponse struct {
	SpatialPoints  [][]float32 `json:"spatial_points"`
	PositionPoints [][]float32 `json:"position_points"`
}

func points(c echo.Context) error {
	var pr pointsRequest
	err := c.Bind(&pr)
	if err != nil {
		c.Logger().Errorf("Invalid collect request body: %s", err)
		return echo.NewHTTPError(http.StatusBadRequest, "Invalid request items")
	}

	err = c.Validate(&pr)
	if err != nil {
		c.Logger().Errorf("Failed validation: %s", err)
	}

	response := pointsResponse{}
	if pr.Mock {
		response.SpatialPoints = [][]float32{{5, 5}, {5, 6}, {5, 8}}
		response.PositionPoints = [][]float32{{1, 2}, {1, 3}, {1, 7}}
	} else {
		board, ok := boards[pr.BoardID]
		if !ok {
			c.Logger().Errorf("Invalid board id used: %s", err)
			return echo.NewHTTPError(http.StatusBadRequest, "Invalid request items")
		}
		response.SpatialPoints = board.spatialPoints
		response.PositionPoints = board.positionPoints
	}
	c.JSON(http.StatusOK, response)

	return nil
}
