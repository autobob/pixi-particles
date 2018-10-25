"use strict";

import ParticleUtils, {Color, SimpleEase} from "./ParticleUtils";
import Particle from "./Particle";
import PropertyNode from "./PropertyNode";

/**
 * A Catmull-Rom spline
 * @memberof PIXI.particles
 * @constructor
 */
export default class CatmullRom {

    /**
     * @property {Array} _controlPoints
     * @private
     */
    protected _controlPoints: Array<PIXI.Point>;

    /**
     * @property {number} _kAlpha
     * @private
     */
    protected _kAlpha:number;

    /**
     * @property {Array} _points
     * @private
     */
    protected _points: Array<PIXI.Point>;

    /**
     * @property {number} _pointsPerSegment
     * @private
     */
    protected _pointsPerSegment:number;

    /**
     * @property {boolean} _dirty
     * @private
     */
    protected _dirty:boolean;

    /**
     * Merges two arrays, removing any duplicate values
     * @method PIXI.particles.CatmullRom#mergeUnique
     */
    public static mergeUnique(...args : any[]) {
        interface hash {
            [key: string] : boolean
        }

        var hash: hash = {},
            ret = [],
            encode, i, j;
        for (i = 0; i < args.length; i++) {
            for (j = 0; j < args[i].length; j++) {
                encode = args[i][j].x + "_" + args[i][j].y;
                if (hash[encode] !== true) {
                    hash[encode] = true;
                    ret[ret.length] = args[i][j];
                }
            }
        }
        return ret;
    }

    /**
     * Calculates the distance between two points.
     * @method PIXI.particles.CatmullRom#pointDistance
     */
    public static pointDistance(a:PIXI.Point, b:PIXI.Point) {
        return Math.sqrt(Math.pow(a.x - b.x, 2) + Math.pow(a.y - b.y, 2));
    }

    /**
     * Calculates the angle between two points.
     * @method PIXI.particles.CatmullRom#pointAngle
     */
    public static pointAngle(a:PIXI.Point, b:PIXI.Point) {
        return Math.atan2(b.x - a.x, b.y - a.y);
    }

    /**
     * returns the point nearest to the input point
     * @method getNearest
     * @param point {PIXI.Point} Point to test
     * @returns {PIXI.Point}
     */
    public getNearest(point:PIXI.Point) {
        var nearest = 0;
        var dist = CatmullRom.pointDistance(point, this._points[0]);
        var newDist;
        for (var i = 0; i < this._points.length; i++) {
            if (this._points[i].x === point.x && this._points[i].y === point.y) {
                return this._points[i];
            } else {
                newDist = CatmullRom.pointDistance(point, this._points[i]);
                if (dist > newDist) {
                    nearest = i;
                    dist = newDist;
                }
            }
        }
        return this._points[nearest];
    };

    /**
     * @method getIndex
     * @description returns the point index at the given progress
     * @param progress {number} Path progress as a fraction
     */
    public getIndex(progress:number) {
        var index = (this._points.length) * (progress % 1);
        if (index < 0) {
            index = this._points.length + index;
        }
        return Math.floor(index);
    };

    /**
     * @method getNormals
     * @description returns the normal vector at the input point (i.e. away from the middle of the shape)
     * @param index {number} Input point index
     */
    public getNormals(index:number) {
        var pointPrev = this._points[index === 0 ? this._points.length - 1 : index - 1];
        var pointNext = this._points[index === this._points.length - 1 ? 0 : index + 1];
        var len = CatmullRom.pointDistance(pointPrev, pointNext);
        var dx = (pointNext.x - pointPrev.x) / len;
        var dy = (pointNext.y - pointPrev.y) / len;

        return {toward: new PIXI.Point(-dy, dx), away: new PIXI.Point(dy, -dx)};
    };

    /**
     * @member {Array} PIXI.particles.CatmullRom#controlPoints
     */
    public get controlPoints() {
        if (this._dirty) {
            this.populate();
        }
        return this._controlPoints;
    };

    public set controlPoints(newVal:Array<PIXI.Point>) {
        this._controlPoints = newVal;
        this._dirty = true;
    };

    /**
     * @member {number} PIXI.particles.CatmullRom#kAlpha
     */
    public get kAlpha() {
        if (this._dirty) {
            this.populate();
        }
        return this._kAlpha;
    };

    public set kAlpha(newVal:number) {
        if (this._kAlpha !== newVal) {
            this._kAlpha = newVal;
            this._dirty = true;
        }
    };

    /**
     * @member {number} PIXI.particles.CatmullRom#pointsPerSegment
     */
    public get pointsPerSegment() {
        if (this._dirty) {
            this.populate();
        }
        return this._pointsPerSegment;
    };

    public set pointsPerSegment(newVal:number) {
        if (this._pointsPerSegment !== newVal) {
            this._pointsPerSegment = newVal;
            this._dirty = true;
        }
    };

    /**
     * @member {Array} PIXI.particles.CatmullRom#points
     */
    public get points() {
        if (this._dirty) {
            this.populate();
        }
        return this._points;
    };

    /**
     * updates the spline points
     * @method PIXI.particles.CatmullRom#populate
     */
    protected populate() {
        var last = this._controlPoints.length - 1;
        this._points = this.catmullRomSpline(this._controlPoints[last], this._controlPoints[0], this._controlPoints[1], this._controlPoints[2]);
        for (var i = 1; i < last - 1; i++) {
            this._points = CatmullRom.mergeUnique(this._points, this.catmullRomSpline(this._controlPoints[i - 1], this._controlPoints[i], this._controlPoints[i + 1], this._controlPoints[i + 2]));
        }
        this._points = CatmullRom.mergeUnique(
            this._points,
            this.catmullRomSpline(this._controlPoints[last - 2], this._controlPoints[last - 1], this._controlPoints[last], this._controlPoints[0]),
            this.catmullRomSpline(this._controlPoints[last - 1], this._controlPoints[last], this._controlPoints[0], this._controlPoints[1])
        );
        this._dirty = false;
    }

    /**
     * @method PIXI.particles.CatmullRom#catmullRomSpline
     * @description Generates a series of points defining a curve between points P1 and P2.
     * @param P0 {PIXI.Point} previous point in series
     * @param P1 {PIXI.Point} first point in spline
     * @param P2 {PIXI.Point} end point in spline
     * @param P3 {PIXI.Point} next point in series
     */
    protected catmullRomSpline(P0:PIXI.Point, P1:PIXI.Point, P2:PIXI.Point, P3:PIXI.Point) {
        //Calculate knots
        var t0: number,
            t1: number,
            t2: number,
            t3: number;

        function tNext(tPrev: number, pPrev: PIXI.Point, pNext: PIXI.Point, kAlpha: number) {
            return Math.pow(Math.sqrt(Math.pow(pNext.x - pPrev.x, 2) + Math.pow(pNext.y - pPrev.y, 2)), kAlpha) + tPrev;
        }

        t0 = 0;
        t1 = tNext(t0, P0, P1, this._kAlpha);
        t2 = tNext(t1, P1, P2, this._kAlpha);
        t3 = tNext(t2, P2, P3, this._kAlpha);

        //get intervals between knots
        function linSpace(t1: number, t2: number, numKnots: number) {
            var ret = [t1];
            var dist = (t2 - t1) / (numKnots - 1);
            while (ret.length < numKnots - 1) {
                ret[ret.length] = ret[ret.length - 1] + dist;
            }
            ret[ret.length] = t2;
            return ret;
        }

        /**
         * @function AN
         * @description calculates the "A" spline equations
         * @param t {number} Position on the spline between t1 and t2
         * @param tP {number} Previous knot
         * @param tN {number} Next knot
         * @param PP {PIXI.Point} Previous control point
         * @param PN {PIXI.Point} Next control point
         */
        function AN(t: number, tP: number, tN: number, PP: PIXI.Point, PN: PIXI.Point) {
            var x = ((tN - t) / (tN - tP) * PP.x) + ((t - tP) / (tN - tP) * PN.x);
            var y = ((tN - t) / (tN - tP) * PP.y) + ((t - tP) / (tN - tP) * PN.y);
            return new PIXI.Point(x, y);
        }

        /**
         * @function BN
         * @description calculates the "B" spline equations
         * @param t {number} Position on the spline between t1 and t2
         * @param tP {number} Previous knot
         * @param tN {number} Next knot
         * @param AP {PIXI.Point} result of AN(t, t(N-1), t(N))
         * @param AN {PIXI.Point} result of AN(t, t(N), t(N+1))
         */
        function BN(t: number, tP: number, tN: number, AP: PIXI.Point, AN: PIXI.Point) {
            var x = ((tN - t) / (tN - tP) * AP.x) + ((t - tP) / (tN - tP) * AN.x);
            var y = ((tN - t) / (tN - tP) * AP.y) + ((t - tP) / (tN - tP) * AN.y);
            return new PIXI.Point(x, y);
        }

        /**
         * @function C
         * @description calculates the "C" spline equation (C  = (t2-t)/(t2-t1)*B1 + (t-t1)/(t2-t1)*B2)
         * @param t {number} Position on the spline between t1 and t2
         */
        function C(t: number) {
            var A1 = AN(t, t0, t1, P0, P1);
            var A2 = AN(t, t1, t2, P1, P2);
            var A3 = AN(t, t2, t3, P2, P3);
            var B1 = BN(t, t0, t2, A1, A2);
            var B2 = BN(t, t1, t3, A2, A3);
            var x = (((t2 - t) / (t2 - t1)) * B1.x) + ((t - t1) / (t2 - t1) * B2.x);
            var y = (((t2 - t) / (t2 - t1)) * B1.y) + ((t - t1) / (t2 - t1) * B2.y);
            return new PIXI.Point(x, y);
        }

        var segments = linSpace(t1, t2, this._pointsPerSegment);
        var ret: Array<PIXI.Point> = [];
        for (var i = 0; i < segments.length; i++) {
            ret[i] = C(segments[i]);
        }
        return ret;
    }

    /**
     * @function CatmullRom
     * @Description helper function for creating/storing a set of points defining a smooth path
     * @param controlPoints {Array<PIXI.Point>}
     * @param [kAlpha = 0.5] {number} knot parameterisation. 0 = uniform/tight, 0.5 = centripetal/default, 1.0 = chordal/loose.
     * @param [pointsPerSegment = 50] {number} Total points generated across any two control points.
     * @constructor
     */
    constructor(controlPoints:Array<PIXI.Point>, kAlpha: number, pointsPerSegment: number) {
        this._controlPoints = controlPoints;
        this._kAlpha = (kAlpha == 0 ? 0 : (kAlpha || 0.5));
        this._pointsPerSegment = pointsPerSegment || 50;
        this._points = [];
        this._dirty = true;
    }
}

