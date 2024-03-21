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
		response.SpatialPoints = [][]float32{{0, 0}, {2, 0.5}, {4, -0.2}, {6, 0.3}, {8, -0.1}, {10, 0}, {9.8, 2}, {10.2, 4}, {9.9, 6}, {10.1, 8}, {10, 10}, {8, 9.5}, {6, 10.2}, {4, 9.8}, {2, 10.1}, {0, 10}, {0.2, 8}, {-0.2, 6}, {0.1, 4}, {-0.1, 2}, {0, 0}}
		response.PositionPoints = [][]float32{{1, 1}, {1, 3}, {3, 3}, {3, 6}, {5, 6}, {5, 9}, {7, 9}, {7, 7}, {9, 7}}
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
