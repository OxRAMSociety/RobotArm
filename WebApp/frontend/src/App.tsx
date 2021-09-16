import React from "react";
import "./App.css";
import Homepage from "./pages/homepage/homepage";
import {
  BrowserRouter as Router,
  Switch,
  Route,
  Link,
  useParams,
} from "react-router-dom";
import VoiceCommandPage from "./pages/voicecommand/VoiceCommandPage";

function App() {
  return (
    <div>
      <Router>
        <Switch>
          <Route exact path="/">
            <Homepage />
          </Route>
          <Route path="/voice-command">
            <VoiceCommandPage />
          </Route>
        </Switch>
      </Router>
    </div>
  );
}

export default App;
