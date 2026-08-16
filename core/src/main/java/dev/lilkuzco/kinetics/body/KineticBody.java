package dev.lilkuzco.kinetics.body;

import dev.lilkuzco.kinetics.aero.Aerodynamics;
import dev.lilkuzco.kinetics.constants.Constants;
import dev.lilkuzco.kinetics.event.EventSink;
import dev.lilkuzco.kinetics.event.KineticEvent;
import dev.lilkuzco.kinetics.math.Quat;
import dev.lilkuzco.kinetics.math.Vec3;
import dev.lilkuzco.kinetics.phase.FlightPhase;
import dev.lilkuzco.kinetics.phase.PhaseMachine;
import dev.lilkuzco.kinetics.profile.Profile;
import dev.lilkuzco.kinetics.profile.Recovery;
import dev.lilkuzco.kinetics.profile.Stage;

/**
 * One simulated body: where it is, how fast, which way it points, how much fuel is left.
 *
 * <p>Mass is <b>derived, never stored</b>. {@link #mass()} adds up the payload, the current
 * stage's structure and remaining propellant, and every stage still stacked above. That is
 * how invariant I4 becomes structural rather than a check: there is no mass field to
 * accidentally assign, so mass can only fall by burning declared fuel or by shedding a
 * declared stage, and dry mass is the floor because the sum bottoms out at the payload.
 */
public final class KineticBody {

    private final String id;
    private final Profile profile;
    private final Constants k;
    private final Aerodynamics aero;
    private final PhaseMachine phases;
    private final dev.lilkuzco.kinetics.profile.EngineFrame engineFrame;

    private Vec3 position;
    private Vec3 velocity;
    private Quat orientation;
    private Vec3 angularVelocity = Vec3.ZERO;

    private int stageIndex;
    private double stageFuel;
    private double throttle = 1.0;
    private double age;

    // Derived flight state, refreshed each substep. Read by consumers for display and by the
    // test battery for assertions; never used as simulation input.
    private double dynamicPressure;
    private double machNumber;
    private double angleOfAttackDeg;
    private double heatingRate;
    private double peakHeatingRate;
    private boolean stalled;
    private double achievedDeltaV;

    private int chutesDeployed;
    private double activeChuteCd;
    private double activeChuteArea;
    private boolean structuralLimitFlagged;
    private boolean overheatFlagged;

    public KineticBody(String id, Profile profile, Constants k,
                       Vec3 position, Vec3 velocity, Quat orientation,
                       FlightPhase initialPhase) {
        this.id = id;
        this.profile = profile;
        this.k = k;
        this.engineFrame = dev.lilkuzco.kinetics.profile.EngineFrame.of(k);
        this.aero = profile.airframe().aerodynamics(k);
        this.position = position;
        this.velocity = velocity;
        this.orientation = orientation.renormalized();
        this.phases = new PhaseMachine(id, initialPhase);
        this.stageIndex = 0;
        this.stageFuel = profile.stages().isEmpty() ? 0.0 : profile.stages().get(0).fuelMass();
    }

    /** An unpowered body: a shell, a capsule, debris. Starts in DESCENT. */
    public static KineticBody unpowered(String id, Profile profile, Constants k,
                                        Vec3 position, Vec3 velocity) {
        Quat facing = velocity.lengthSq() > 1e-12
                ? Quat.between(new Vec3(0, 0, 1), velocity.normalized())
                : Quat.IDENTITY;
        return new KineticBody(id, profile, k, position, velocity, facing, FlightPhase.DESCENT);
    }

    // ---- mass and fuel (I4) ----------------------------------------------

    /** Current total mass, kg. Derived from stage state every time it is asked for. */
    public double mass() {
        double m = profile.payloadDryMass();
        var stages = profile.stages();
        for (int i = stageIndex + 1; i < stages.size(); i++) {
            m += stages.get(i).wetMass();
        }
        if (stageIndex < stages.size()) {
            m += stages.get(stageIndex).stageDryMass() + stageFuel;
        }
        return m;
    }

    /** The mass floor: what remains when every stage is spent and shed. */
    public double dryMassFloor() { return profile.payloadDryMass(); }

    public boolean hasFuel() { return stageFuel > 0.0; }

    public double stageFuel() { return stageFuel; }

    public int stageIndex() { return stageIndex; }

    public Stage currentStage() {
        var stages = profile.stages();
        return stageIndex < stages.size() ? stages.get(stageIndex) : null;
    }

    public boolean hasStagesRemaining() { return stageIndex < profile.stages().size(); }

    /**
     * Burn propellant for {@code dt} at the current throttle and return the thrust magnitude
     * actually produced, N.
     *
     * <p>The single door of RD2: mass flow comes from {@code F_vac/(Isp_vac*g0)}, fuel comes
     * off at that rate, and thrust is that same flow times the <em>altitude-corrected</em> Isp
     * (RD2b). Nothing else in the library may reduce mass.
     *
     * @param pressureRatio ambient pressure over sea level, for the Isp interpolation
     */
    public double burn(double dt, double pressureRatio) {
        Stage stage = currentStage();
        if (stage == null || stageFuel <= 0.0 || throttle <= 0.0) return 0.0;

        double mdot = stage.massFlow(engineFrame) * throttle;
        double burned = mdot * dt;
        if (burned > stageFuel) {
            // Partial substep at burnout: scale the impulse to the propellant that actually
            // existed, rather than letting the last step spend fuel the stage did not have.
            burned = stageFuel;
            mdot = burned / dt;
        }
        stageFuel -= burned;
        if (stageFuel < 1e-12) stageFuel = 0.0;

        return mdot * stage.effectiveExhaustVelocity(pressureRatio, engineFrame);
    }

    public dev.lilkuzco.kinetics.profile.EngineFrame engineFrame() { return engineFrame; }

    /**
     * Shed the spent stage and light the next (RD4). Returns false when nothing is left.
     * The mass ratio of everything above improves the instant the structure goes.
     */
    public boolean advanceStage(EventSink events) {
        var stages = profile.stages();
        if (stageIndex >= stages.size()) return false;
        double shed = stages.get(stageIndex).stageDryMass();
        events.accept(new KineticEvent.Staging(id, age, stageIndex, shed, velocity.length()));
        stageIndex++;
        stageFuel = stageIndex < stages.size() ? stages.get(stageIndex).fuelMass() : 0.0;
        return stageIndex < stages.size();
    }

    // ---- parachutes (RB6) ------------------------------------------------

    /**
     * Try to open the next chute in sequence. Above its deploy limit it shreds instead, and
     * kinetics reports that and moves on - what a shredded chute means is the consumer's call
     * (I10).
     *
     * @return true if a canopy is now inflated
     */
    public boolean tryDeployNextChute(double altitude, double q, EventSink events) {
        Recovery recovery = profile.recovery();
        if (chutesDeployed >= recovery.chutes().size()) return false;
        Recovery.Parachute chute = recovery.chutes().get(chutesDeployed);
        if (altitude > chute.deployAltitude()) return false;

        chutesDeployed++;
        if (!chute.survivesDeployAt(q)) {
            events.accept(new KineticEvent.ChuteShred(id, age, chute.name(), q, chute.qDeployMax()));
            return activeChuteArea > 0.0;
        }
        activeChuteCd = chute.cd();
        activeChuteArea = chute.area();
        events.accept(new KineticEvent.ChuteDeployed(id, age, chute.name(), altitude, q));
        return true;
    }

    public boolean hasInflatedChute() { return activeChuteArea > 0.0; }

    /**
     * The canopy's drag area product {@code C_d * A}, m^2. Drag scales with this product, so
     * carrying it as one number keeps a small capsule under a large canopy correct - averaging
     * the two coefficients separately would badly understate the chute.
     */
    public double chuteCdA() { return activeChuteCd * activeChuteArea; }

    /** Effective drag area including any inflated canopy, m^2. */
    public double effectiveDragArea() {
        return profile.airframe().referenceArea() + activeChuteArea;
    }

    /**
     * Reference-area-weighted C_d including any canopy. Combining as {@code C_d*A} and
     * dividing back out keeps the drag force correct when a small body hangs under a large
     * canopy - averaging the coefficients alone would badly understate the chute.
     */
    public double blendedCd(double bodyCd) {
        double bodyArea = profile.airframe().referenceArea();
        double total = bodyArea + activeChuteArea;
        if (total <= 0.0) return bodyCd;
        return (bodyCd * bodyArea + activeChuteCd * activeChuteArea) / total;
    }

    // ---- accessors -------------------------------------------------------

    public String id() { return id; }

    public Profile profile() { return profile; }

    public Aerodynamics aerodynamics() { return aero; }

    public PhaseMachine phases() { return phases; }

    public FlightPhase phase() { return phases.phase(); }

    public Vec3 position() { return position; }

    public Vec3 velocity() { return velocity; }

    public Quat orientation() { return orientation; }

    public Vec3 angularVelocity() { return angularVelocity; }

    public double age() { return age; }

    public double throttle() { return throttle; }

    public double speed() { return velocity.length(); }

    public void setPosition(Vec3 p) { this.position = p; }

    public void setVelocity(Vec3 v) { this.velocity = v; }

    public void setOrientation(Quat q) { this.orientation = q.renormalized(); }

    public void setAngularVelocity(Vec3 w) { this.angularVelocity = w; }

    public void setThrottle(double t) {
        this.throttle = t < 0.0 ? 0.0 : Math.min(t, 1.0);
    }

    public void advanceAge(double dt) { this.age += dt; }

    public void addAchievedDeltaV(double dv) { this.achievedDeltaV += dv; }

    /** Delta-v actually produced by thrust so far, m/s. Checked against Tsiolkovsky (I4). */
    public double achievedDeltaV() { return achievedDeltaV; }

    public void recordFlightState(double q, double mach, double aoaDeg, boolean stalled) {
        this.dynamicPressure = q;
        this.machNumber = mach;
        this.angleOfAttackDeg = aoaDeg;
        this.stalled = stalled;
    }

    public void recordHeating(double rate) {
        this.heatingRate = rate;
        if (rate > peakHeatingRate) peakHeatingRate = rate;
    }

    public double dynamicPressure() { return dynamicPressure; }

    public double machNumber() { return machNumber; }

    public double angleOfAttackDeg() { return angleOfAttackDeg; }

    public boolean isStalled() { return stalled; }

    /** Current stagnation-point heating rate, W/m^2 (RE7). A field, never a damage source. */
    public double heatingRate() { return heatingRate; }

    public double peakHeatingRate() { return peakHeatingRate; }

    public boolean structuralLimitFlagged() { return structuralLimitFlagged; }

    public void flagStructuralLimit() { this.structuralLimitFlagged = true; }

    public boolean overheatFlagged() { return overheatFlagged; }

    public void flagOverheat() { this.overheatFlagged = true; }

    public int chutesDeployed() { return chutesDeployed; }

    /** Specific mechanical energy, J/kg: {@code v^2/2 + g*h}. The quantity I3 watches. */
    public double specificEnergy(double gravity, double altitude) {
        return 0.5 * velocity.lengthSq() + gravity * altitude;
    }

    /** Immutable snapshot, for golden-trajectory hashing and for client interpolation. */
    public BodyState snapshot() {
        return new BodyState(id, age, position, velocity, orientation, angularVelocity,
                mass(), stageIndex, stageFuel, phases.phase(), dynamicPressure, machNumber,
                angleOfAttackDeg, heatingRate);
    }

    @Override
    public String toString() {
        return String.format("%s[%s t=%.2fs pos=%s v=%.1fm/s m=%.1fkg stage=%d fuel=%.1fkg]",
                id, phases.phase(), age, position, speed(), mass(), stageIndex, stageFuel);
    }
}
