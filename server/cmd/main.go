package main

import (
	"net/http"

	"github.com/DS1S/simple-slam/internal/api"
	"github.com/go-playground/validator"
	"github.com/labstack/echo/v4"
	"github.com/labstack/echo/v4/middleware"
)

func main() {
	e := echo.New()
	e.Use(middleware.Logger())
	WithValidator(e)

	api.WithRoutes(e)
	e.Start(":3000")
}

func WithValidator(e *echo.Echo) {
	e.Validator = &Validator{validator: validator.New()}
}

type Validator struct {
	validator *validator.Validate
}

func (cv *Validator) Validate(i interface{}) error {
	if err := cv.validator.Struct(i); err != nil {
		return echo.NewHTTPError(http.StatusBadRequest, err.Error())
	}
	return nil
}
