package api

import (
	"net/http"

	"github.com/labstack/echo/v4"
)

type resetParams struct {
	BoardID boardID `param:"board_id" validate:"required"`
}

func reset(c echo.Context) error {
	var rr resetParams

	err := c.Bind(&rr)
	if err != nil {
		c.Logger().Errorf("Could not bind resetRequest body: %s", err)
		echo.NewHTTPError(http.StatusBadRequest, "Invalid request")
	}

	err = c.Validate(&rr)
	if err != nil {
		return err
	}

	delete(boards, rr.BoardID)

	return c.JSON(http.StatusOK, map[string]interface{}{
		"msg": "Succesfully reseted board",
	})
}
