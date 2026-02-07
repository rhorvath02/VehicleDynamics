import { useEffect } from "react";

export default function Metrics() {
    useEffect(() => {
        if ((window as any).MathJax) {
            (window as any).MathJax.typesetPromise();
        }
    }, []);

  return (
    <div className="page">
      <h1>Vehicle Performance Metrics</h1>

      <p>
        This page provides conceptual definitions of commonly used vehicle
        performance metrics. Each metric is described in terms of the physical
        behavior it represents and why it is relevant to vehicle design and
        analysis. No assumptions are made regarding relative importance; the
        relevance of each metric depends on the intended application of the
        vehicle.
      </p>

      {/* ============================= */}
      {/* Steady-State Handling Metrics */}
      {/* ============================= */}

      <h2>Steady-State Handling</h2>
      <p className="section-intro">
        Metrics describing equilibrium behavior once transient effects have
        decayed.
      </p>

      <section>
        <h3>Maximum Lateral Acceleration (Max&nbsp;Ay)</h3>

        <p>
          <strong>Description.</strong> The maximum lateral acceleration the
          vehicle can sustain in a steady-state corner before tire saturation or
          loss of stability.
        </p>

        <p>
          <strong>Physical interpretation.</strong> This metric reflects the
          combined lateral force-generating capability of the tires, suspension
          geometry, load transfer characteristics, and overall vehicle mass
          distribution under quasi-static conditions.
        </p>

        <p>
          <strong>Why it matters.</strong> Maximum lateral acceleration is
          commonly used as a proxy for absolute steady-state cornering capability
          and provides a compact measure of how effectively the vehicle can
          generate lateral force.
        </p>

        <p>
          In simplified form,
        </p>

        <p className="math">
          {"\\[ a_y = \\frac{v^2}{R} \\]"}
        </p>

        <p>
          where the achievable value of <span className="math-inline">{"\\(a_y\\)"}</span> is limited by the
          available tire lateral force.
        </p>
      </section>

      <section>
        <h3>Linear Understeer Gradient</h3>

        <p>
          <strong>Description.</strong> The rate at which required steering input
          increases with lateral acceleration in the linear tire operating
          region.
        </p>

        <p>
          <strong>Physical interpretation.</strong> The linear understeer
          gradient characterizes the balance between front and rear cornering
          stiffness and is influenced by suspension geometry, roll stiffness
          distribution, tire properties, and compliance.
        </p>

        <p>
          <strong>Why it matters.</strong> This metric defines the fundamental
          handling balance perceived by the driver during moderate cornering and
          strongly influences predictability and steering effort.
        </p>

        <p className="math">
          {"\\[ K = \\frac{\\partial \\delta}{\\partial a_y} \\]"}
        </p>

        <p>
          where <span className="math-inline">{"\\(\\delta\\)"}</span> is steering angle and
          <span className="math-inline">{"\\(a_y\\)"}</span> is lateral acceleration.
        </p>
      </section>

      <section>
        <h3>Limit Understeer Behavior</h3>

        <p>
          <strong>Description.</strong> The effective understeer or oversteer
          tendency as the vehicle approaches tire saturation.
        </p>

        <p>
          <strong>Physical interpretation.</strong> This behavior arises from
          nonlinear tire force characteristics, load transfer effects, and
          compliance within the suspension and steering systems.
        </p>

        <p>
          <strong>Why it matters.</strong> Limit understeer behavior governs how
          the vehicle behaves near the edge of grip, influencing controllability
          and driver confidence during aggressive maneuvers.
        </p>
      </section>

      <section>
        <h3>Linear Sideslip Gradient</h3>

        <p>
          <strong>Description.</strong> The relationship between vehicle sideslip
          angle and lateral acceleration in the linear operating region.
        </p>

        <p>
          <strong>Physical interpretation.</strong> This metric reflects the yaw
          stiffness of the vehicle and is influenced by mass distribution, yaw
          inertia, and tire slip angle characteristics.
        </p>

        <p>
          <strong>Why it matters.</strong> Linear sideslip behavior affects how
          stable or “planted” the vehicle feels during normal cornering.
        </p>

        <p className="math">
          {"\\[ \\beta = f(a_y) \\]"}
        </p>
      </section>

      <section>
        <h3>Limit Sideslip Behavior</h3>

        <p>
          <strong>Description.</strong> Vehicle sideslip response as the tires
          approach or exceed their lateral force limits.
        </p>

        <p>
          <strong>Physical interpretation.</strong> Governed by yaw moment
          balance, tire force saturation and fall-off, and inertial coupling.
        </p>

        <p>
          <strong>Why it matters.</strong> Limit sideslip behavior determines
          whether the vehicle remains controllable at the edge of grip or
          transitions abruptly into loss of control.
        </p>
      </section>

      {/* ========================= */}
      {/* Transient Handling */}
      {/* ========================= */}

      <h2>Transient Handling</h2>
      <p className="section-intro">
        Metrics describing time-domain response to steering inputs.
      </p>

      <section>
        <h3>Yaw Rate Gain</h3>

        <p>
          <strong>Description.</strong> The ratio between vehicle yaw rate and
          steering input under steady or low-frequency conditions.
        </p>

        <p className="math">
          {"\\[ G_r = \\frac{r}{\\delta} \\]"}
        </p>

        <p>
          <strong>Why it matters.</strong> Yaw rate gain defines how strongly the
          vehicle responds rotationally to driver input and forms the basis for
          interpreting many other dynamic metrics.
        </p>
      </section>

      <section>
        <h3>Yaw Rate Response Delay</h3>

        <p>
          <strong>Description.</strong> The time or phase delay between steering
          input and yaw rate response.
        </p>

        <p className="math">
          {"\\[ \\phi_r(\\omega) = \\angle \\frac{R(\\omega)}{\\Delta(\\omega)} \\]"}
        </p>

        <p>
          <strong>Why it matters.</strong> Yaw delay strongly affects perceived
          responsiveness and driver confidence during rapid maneuvers.
        </p>
      </section>

      <section>
        <h3>Lateral Acceleration Response Delay</h3>

        <p>
          <strong>Description.</strong> The time or phase delay between steering
          input and lateral acceleration response.
        </p>

        <p>
          <strong>Why it matters.</strong> This metric influences how quickly the
          vehicle “takes a set” and how connected it feels during turn-in.
        </p>
      </section>

      <section>
        <h3>Yaw Rate Amplification</h3>

        <p>
          <strong>Description.</strong> The ratio of peak yaw rate response to
          steady-state yaw rate response following a steering input.
        </p>

        <p className="math">
          {"\\[ A_r = \\frac{r_{peak}}{r_{ss}} \\]"}
        </p>

        <p>
          <strong>Why it matters.</strong> Yaw amplification indicates whether
          the vehicle exhibits aggressive transient overshoot or a controlled,
          well-damped response.
        </p>
      </section>

      {/* ========================= */}
      {/* Stability */}
      {/* ========================= */}

      <h2>Stability and Control</h2>

      <section>
        <h3>Stability Margin</h3>

        <p>
          <strong>Description.</strong> A measure of the available yaw moment or
          control authority before the onset of instability.
        </p>

        <p>
          <strong>Physical interpretation.</strong> Stability margin reflects
          tire force reserves, load transfer balance, yaw inertia, and overall
          system robustness.
        </p>

        <p>
          <strong>Why it matters.</strong> This metric provides insight into how
          much corrective authority exists before loss of control occurs.
        </p>
      </section>

      {/* ========================= */}
      {/* Frequency Domain */}
      {/* ========================= */}

      <h2>Frequency-Domain Metrics</h2>

      <section>
        <h3>Yaw Rate Frequency Response</h3>

        <p>
          <strong>Description.</strong> The frequency-dependent magnitude and
          phase relationship between steering input and yaw rate response.
        </p>

        <p className="math">
          {"\\[ G_r(j\\omega) = \\frac{R(j\\omega)}{\\Delta(j\\omega)} \\]"}
        </p>

        <p>
          <strong>Why it matters.</strong> Frequency-domain metrics provide a
          compact representation of vehicle dynamics across a wide range of time
          scales.
        </p>
      </section>

      <section>
        <h3>Lateral Acceleration Frequency Response</h3>

        <p>
          <strong>Description.</strong> The frequency-dependent relationship
          between steering input and lateral acceleration under representative
          operating conditions.
        </p>

        <p>
          <strong>Why it matters.</strong> Evaluating frequency response under
          realistic loading provides insight into dynamic behavior not captured
          by purely linear analysis.
        </p>
      </section>

      <p className="closing-note">
        These metrics collectively describe vehicle capability, response, and
        stability. Their relevance depends on the intended application, operating
        environment, and design objectives.
      </p>
    </div>
  );
}
