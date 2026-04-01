import { render, screen } from "@testing-library/react";
import App from "./App";

describe("App", () => {
  beforeEach(() => {
    global.fetch = jest.fn();
  });

  afterEach(() => {
    jest.resetAllMocks();
  });

  test("renders the loading state before telemetry arrives", () => {
    render(<App />);
    expect(screen.getByText(/connecting to telemetry/i)).toBeInTheDocument();
  });
});
