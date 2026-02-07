import { createBrowserRouter } from "react-router-dom";
import AppLayout from "./layouts/AppLayout";
import Home from "./pages/Home";
import Metrics from "./pages/Metrics";

export const router = createBrowserRouter(
  [
    {
      path: "/",
      element: <AppLayout />,
      children: [
        { index: true, element: <Home /> },
        { path: "metrics", element: <Metrics /> }
      ]
    }
  ],
  {
    basename: import.meta.env.BASE_URL
  }
);
