/*jshint esversion: 6*/

// Field geometry
var field = {
    length: 9.0,
    width: 6.0,
    borders: 0.5,
    centerCircleRadius: 1,
    goalQuarterCircleRadius: 1,
    goalLinearWidth: 0.5,
    goalWidth: 1,
    goalDepth: 0.18,
    goalThickness: 0.023,
    linesThickness: 0.02
};

// Robots
var robots = [
    {
        id: 1,
        x: -2,
        y: 2,
        orientation: 0,
        color: 'blue',
        present: true
    },
    {
        id: 2,
        x: -2,
        y: -2,
        orientation: 0,
        color: 'blue',
        present: true
    },
    {
        id: 2,
        x: 2,
        y: 2,
        orientation: Math.PI,
        color: 'yellow',
        present: false
    },
    {
        id: 3,
        x: 2,
        y: -1,
        orientation: Math.PI/1.5,
        color: 'yellow',
        present: true
    }
];

// Our team color
var ourColor = 'yellow';

// Ball
var ball = [0, 0];

class Viewer
{
    constructor()
    {
        this.zone = $('.field-zone');
        this.container = null;
        this.ctx = null;
        this.mousePos = null;
        this.ratio = null;
        this.dragging = false;
        this.dragBegin = null;
        this.viewOffset = [0, 0];
        this.startOffset = [0, 0];
        this.reversed = false;
        this.greenred = false;

        this.container = document.getElementById('field');
        this.ctx = this.container.getContext('2d');

        setInterval(() => this.update(), 20);
        this.container.addEventListener('mousemove', (evt) => this.mouseMove(evt));
        this.container.addEventListener('mousedown', (evt) => this.mouseDown(evt));
        this.container.addEventListener('mouseup', (evt) => this.mouseUp(evt));
        this.container.addEventListener('DOMMouseScroll', (evt) => this.mouseWheel(evt));
        this.container.addEventListener('mousewheel', (evt) => this.mouseWheel(evt));
        document.addEventListener('keypress', (evt) => this.keyPress(evt));
    }

    mouseMove(evt)
    {
        this.mousePos = [evt.clientX, evt.clientY];
    }

    // Compute the mouse position un meter
    mousePosMeters()
    {
        var left = (this.width-(field.length/this.ratio))/2;
        var top = (this.height-(field.width/this.ratio))/2;
        var X = ((this.mousePos[0]-left)/this.ratio);
        var Y = -((this.mousePos[1]-top)/this.ratio);
        X -= this.viewOffset[0];
        Y -= this.viewOffset[1];

        return [X, Y];
    }

    // Resetting the ratio to fit the canvas zone
    resetRatio()
    {
        var ratio2 = this.height/(field.width + 2*field.borders);
        var ratio1 = this.width/(field.length + 2*field.borders);
        if (ratio2 < ratio1) this.ratio = ratio2;
        else this.ratio = ratio1;
    }

    grColor(color)
    {
        if (!this.greenred) {
            if (color == 'yellow') {
                return '#ede370';
            } else {
                return '#70d1ed';
            }
        } else {
            if (color == ourColor) {
                return '#83e568';
            } else {
                return '#e56868';
            }
        }
    }

    update()
    {
        var ctx = this.ctx;

        // Retrieving zone sizes
        this.width = this.zone.width();
        this.height = this.zone.height();
        this.container.width = this.width;
        this.container.height = this.height;

        // ctx.clearRect(0, 0, 100, 100);
        ctx.clearRect(0, 0, this.width, this.height);

        // Translating to the middle, rescaling for meters
        ctx.save();
        if (this.ratio == null) {
            this.resetRatio();
        }

        ctx.fillStyle = '#aaa';
        ctx.font = '12pt sans';
        if (this.mousePos) {
            var pos = this.mousePosMeters();

            ctx.fillText("X: "+pos[0].toFixed(2)+"m, Y: "+pos[1].toFixed(2)+"m", 10, 20);

            // Moving the view offset
            if (this.dragging) {
                var dx = (this.mousePos[0] - this.dragBegin[0])/this.ratio;
                var dy = -(this.mousePos[1]- this.dragBegin[1])/this.ratio;
                this.viewOffset = [this.startOffset[0] + dx, this.startOffset[1] + dy];
            }
        }

        // Translating to the center
        ctx.translate(this.width/2, this.height/2);
        // Rescaling using ratio
        ctx.scale(this.ratio, -this.ratio);
        // Translating to the view offset
        ctx.translate(this.viewOffset[0], this.viewOffset[1]);

        // Reverse
        if (this.reversed) {
            this.sign = -1;
        } else {
            this.sign = 1;
        }

        // Drawing the field
        this.drawField();

        // Drawing robots
        this.drawRobots();

        // Drawing the ball
        this.drawBall();

        ctx.restore();
    }

    drawField()
    {
        var ctx = this.ctx;

        // Border lines
        ctx.strokeStyle = '#aaa';
        ctx.lineWidth = field.linesThickness;
        ctx.strokeRect(-field.length/2, -field.width/2, field.length, field.width);

        // Lines
        ctx.beginPath();
        ctx.moveTo(0, field.width/2);
        ctx.lineTo(0, -field.width/2);
        ctx.stroke();

        ctx.beginPath();
        ctx.moveTo(-field.length/2, 0);
        ctx.lineTo(field.length/2, 0);
        ctx.stroke();

        // Center circle
        ctx.beginPath();
        ctx.arc(0, 0, field.centerCircleRadius, 0, Math.PI*2);
        ctx.stroke();

        function drawGoals(ctx, color) {
            // Goals zone
            ctx.strokeStyle = '#aaa';
            ctx.lineWidth = field.linesThickness;
            ctx.beginPath();
            ctx.arc(-field.length/2, -field.goalLinearWidth/2, field.goalQuarterCircleRadius, -Math.PI/2, 0);
            ctx.lineTo(-(field.length/2)+field.goalQuarterCircleRadius, field.goalLinearWidth/2);
            ctx.arc(-field.length/2, field.goalLinearWidth/2, field.goalQuarterCircleRadius, 0, Math.PI/2);
            ctx.stroke();

            // Goals
            ctx.strokeStyle = color;
            ctx.lineWidth = field.goalThickness;
            ctx.beginPath();
            ctx.moveTo(-field.length/2, field.goalWidth/2);
            ctx.lineTo(-field.length/2 - field.goalDepth, field.goalWidth/2);
            ctx.lineTo(-field.length/2 - field.goalDepth, -field.goalWidth/2);
            ctx.lineTo(-field.length/2, -field.goalWidth/2);
            ctx.stroke();
        }

        // Drawing (symmetrically) goals
        drawGoals(ctx, this.reversed ? this.grColor('blue') : this.grColor('yellow'));
        ctx.save();
        ctx.scale(-1, 1);
        drawGoals(ctx, this.reversed ? this.grColor('yellow') : this.grColor('blue'));
        ctx.restore();
    }

    drawRobots()
    {
        var ctx = this.ctx;

        for (var k in robots) {
            this.drawRobot(robots[k]);
        }
    }

    drawRobot(robot)
    {
        var ctx = this.ctx;
        var front = 0.75;

        ctx.save();
        ctx.globalAlpha = robot.present ? 1 : 0.5;

        ctx.save();
        if (this.reversed)
        ctx.scale(-1, 1);
        ctx.beginPath();
        ctx.strokeStyle = '#aaa';
        ctx.fillStyle = this.grColor(robot.color);
        ctx.arc(robot.x, robot.y, 0.1, robot.orientation+front, robot.orientation+Math.PI*2-front);
        if (!robot.present) {
            ctx.stroke();
        }
        ctx.fill();

        ctx.restore();
        ctx.save();
        ctx.fillStyle = robot.present ? '#333' : 'white';
        ctx.font = '0.14pt sans';
        ctx.scale(1, -1);
        ctx.fillText(''+robot.id, this.sign*robot.x-0.05, -robot.y+0.065);
        ctx.restore();
        ctx.restore();
    }

    drawBall()
    {
        var ctx = this.ctx;

        ctx.beginPath();
        ctx.fillStyle = 'orange';
        ctx.arc(ball[0], ball[1], 0.043, 0, Math.PI*2);
        ctx.fill();
    }

    // Mouse is down
    mouseDown(evt)
    {
        if (this.mousePos) {
            if (evt.buttons == 4) {
                // Mouse3 to drag the view
                this.dragging = true;
                this.dragBegin = this.mousePos;
                this.startOffset = this.viewOffset;
            }
        }
    }

    // Stopping the dragging
    mouseUp(evt)
    {
        this.dragging = false;
    }

    // Spinning the mouse wheel
    mouseWheel(evt)
    {
        this.ratio -= 5*evt.detail;
        if (this.ratio < 0) {
            this.ratio = 1;
        }
        evt.preventDefault();
    }

    // A key was pressed on the keyboard
    keyPress(evt)
    {
        if (evt.key == 'Escape' || evt.keyCode == 27) {
            this.resetRatio();
            this.viewOffset = [0, 0];
            console.log(evt);
        }
    }
}

class Manager
{
    constructor() {
        var template = $('.robots').html();
        var html = '';

        for (let id=1; id<=8; id++) {
            var robotHtml = template;
            robotHtml = robotHtml.replace(/{{id}}/g, id);
            html += robotHtml;
        }

        $('.robots').html(html);

        $('.robots .expand').click(function() {
            var id = $(this).attr('rel');
            var visible = $('.robot-'+id+' .infos').is(':visible');
            $('.robots .infos').hide();
            $('.robots .icon-collapse').hide();
            $('.robots .icon-expand').show();
            $('.robot').removeClass('expanded');

            if (visible) {
                $('.robot-'+id+' .infos').hide();
            } else {
                $('.robot-'+id).addClass('expanded');
                $('.robot-'+id+' .infos').show();
                $('.robot-'+id+' .icon-collapse').show();
                $('.robot-'+id+' .icon-expand').hide();
            }
        });
    }
}

$(document).ready(function() {
    // Instantiating the viewer
    var viewer = new Viewer();

    // Robots manager
    var manager = new Manager();

    $('.reverse-view').change(function() {
        viewer.reversed = $(this).is(':checked');
    });
    $('.greenred-mode').change(function() {
        viewer.greenred = $(this).is(':checked');
    });
    $('.we-are-color').click(function() {
        if (ourColor == 'yellow') {
            ourColor = 'blue';
        } else {
            ourColor = 'yellow';
        }
        $(this).text('We are: '+ourColor);
    });
});
