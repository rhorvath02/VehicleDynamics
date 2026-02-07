export default function Home() {
  return (
    <div className="page">
      <h1>BobDynamics</h1>

      <p>
        BobDynamics is a reference and analysis framework for understanding,
        defining, and evaluating vehicle behavior. The focus is on first-principles
        physical interpretation of vehicle dynamics rather than test-specific
        procedures or prescriptive design targets.
      </p>

      <p>
        This site documents the conceptual performance metrics used to describe
        vehicle handling, stability, and dynamic response. These metrics are
        intended to form a common language that can be applied across simulation,
        physical testing, and design workflows.
      </p>

      <h2>Design Philosophy</h2>

      <p>
        Rather than prescribing what a vehicle <em>should</em> be optimized for,
        BobDynamics defines what a vehicle <em>does</em> physically. The
        relevance of any given metric depends on the intended application,
        operating environment, and design objectives.
      </p>

      <p>
        By separating physical behavior from prioritization, the same framework
        can be used for motorsport, production vehicles, research, or educational
        purposes without modification.
      </p>

      <h2>What This Site Contains</h2>

      <ul>
        <li>
          Conceptual definitions of vehicle performance metrics and the physical
          behaviors they represent
        </li>
        <li>
          Explanations of why these metrics are commonly used in vehicle design
          and analysis
        </li>
        <li>
          A consistent vocabulary for discussing steady-state, transient, and
          dynamic vehicle behavior
        </li>
      </ul>

      <h2>What This Site Does Not Do</h2>

      <ul>
        <li>Prescribe design targets or numerical thresholds</li>
        <li>Rank metrics by importance</li>
        <li>Recommend specific test procedures or standards</li>
        <li>Provide tuning guidance or control strategies</li>
      </ul>

      <p className="closing-note">
        The intent is to provide a clear, physics-based foundation upon which
        application-specific decisions can be made.
      </p>
    </div>
  );
}
