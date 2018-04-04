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
        present: true,
        enabled: true
    },
    {
        id: 2,
        x: -2,
        y: -2,
        orientation: 0,
        color: 'blue',
        present: true,
        enabled: true
    },
    {
        id: 2,
        x: 2,
        y: 2,
        orientation: Math.PI,
        color: 'yellow',
        present: false,
        enabled: true
    },
    {
        id: 3,
        x: 2,
        y: -1,
        orientation: Math.PI/1.5,
        color: 'yellow',
        present: true,
        enabled: true
    }
];

// Our team color
var ourColor = 'yellow';

// Ball
var ball = [0, 0];

function robotById(id, color)
{
    if (typeof(color) == 'undefined') {
        color = ourColor;
    }

    for (var k in robots) {
        var robot = robots[k];

        if (robot.id == id && robot.color == color) {
            return robot;
        }
    }

    return null;
}

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
        this.draggingRobot = null;
        this.draggingBall = false;
        this.rotatingRobot = null;
        this.dragLock = false;
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

    // Mouse angle since begining of the drag
    mouseAngle(robot)
    {
        if (this.mousePos) {
            var pos = this.mousePosMeters();
            var dx = pos[0] - robot.x;
            var dy = pos[1] - robot.y;

            return Math.atan2(dy, dx);
        }

        return 0;
    }

    // Resetting the ratio to fit the canvas zone
    resetRatio()
    {
        var ratio2 = this.height/(field.width + 2*field.borders);
        var ratio1 = this.width/(field.length + 2*field.borders);
        if (ratio2 < ratio1) this.ratio = ratio2;
        else this.ratio = ratio1;
        this.viewOffset = [0, 0];
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
            let pos = this.mousePosMeters();

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

        if (this.mousePos) {
            let pos = this.mousePosMeters();

            // Dragging a robot
            if (this.draggingRobot) {
                let robot = robotById(this.draggingRobot[0], this.draggingRobot[1]);
                robot = JSON.parse(JSON.stringify(robot));
                robot.ghost = true;
                robot.x = pos[0];
                robot.y = pos[1];
                this.drawRobot(robot);
            }

            // Rotating a robot
            if (this.rotatingRobot) {
                let robot = robotById(this.rotatingRobot[0], this.rotatingRobot[1]);
                robot = JSON.parse(JSON.stringify(robot));

                var angle = this.mouseAngle(robot);
                console.log(angle);

                robot.ghost = true;
                robot.orientation = angle;
                this.drawRobot(robot);
            }
        }

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
        var over = this.overRobot();

        for (var k in robots) {
            var robot = robots[k];
            this.drawRobot(robot, over == robot);
        }
    }

    drawRobot(robot, extra)
    {
        var ctx = this.ctx;
        var front = 0.75;

        if (!('ghost' in robot) && this.rotatingRobot && this.rotatingRobot[0] == robot.id &&
            this.rotatingRobot[1] == robot.color) {
            return;
        }

        ctx.save();
        ctx.globalAlpha = robot.present ? 1 : 0.5;
        if ('ghost' in robot) {
            ctx.globalAlpha = 0.5;
        }

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

        if (typeof(extra) != 'undefined' && extra) {
            ctx.globalAlpha = 1;
            ctx.font = '0.09pt monospace';
            ctx.fillStyle = '#aaa';
            ctx.fillText(robot.color+' #'+robot.id, robot.x+0.15, -robot.y-0.1);
            ctx.fillText('x:'+robot.x.toFixed(2)+'m', robot.x+0.15, -robot.y);
            ctx.fillText('y:'+robot.y.toFixed(2)+'m', robot.x+0.15, -robot.y+0.1);
            ctx.fillText('t:'+(180*robot.orientation/Math.PI).toFixed(1)+'°', robot.x+0.15, -robot.y+0.2);
        }

        ctx.restore();
        ctx.restore();
    }

    drawBall()
    {
        var ctx = this.ctx;

        let pos = JSON.parse(JSON.stringify(ball));
        if (this.draggingBall && this.mousePos) {
            pos = this.mousePosMeters();
        }

        ctx.beginPath();
        ctx.fillStyle = 'orange';
        ctx.arc(pos[0], pos[1], 0.043, 0, Math.PI*2);
        ctx.fill();

        if (this.overBall()) {
            ctx.save();
            ctx.scale(1, -1);
            ctx.font = '0.09pt monospace';
            ctx.fillText('ball', pos[0]+0.15, -pos[1]-0.1);
            ctx.fillText('x:'+pos[0].toFixed(2)+'m', pos[0]+0.15, -pos[1]);
            ctx.fillText('y:'+pos[1].toFixed(2)+'m', pos[0]+0.15, -pos[1]+0.1);
            ctx.restore();
        }
    }

    overBall()
    {
        if (this.mousePos) {
            var xy = this.mousePosMeters();
            var dist = Math.sqrt(Math.pow(ball[0]-xy[0], 2) + Math.pow(ball[1]-xy[1], 2));

            return dist < 0.1;
        }

        return false;
    }

    overRobot()
    {
        if (this.mousePos) {
            var xy = this.mousePosMeters();
            for (var k in robots) {
                var robot = robots[k];
                var dist = Math.sqrt(Math.pow(robot.x-xy[0], 2) + Math.pow(robot.y-xy[1], 2));
                if (dist < 0.1) {
                    return robot;
                }
            }
        }

        return null;
    }

    // Mouse is down
    mouseDown(evt)
    {
        if (this.mousePos) {
            this.dragBegin = this.mousePos;

            if (evt.buttons == 1) {
                if (this.overBall()) {
                    this.draggingBall = true;
                } else {
                    let over = this.overRobot();
                    if (over) {
                        this.draggingRobot = [over.id, over.color];
                    } else if (!this.dragLock) {
                        // Mouse3 to drag the view
                        this.dragging = true;
                        this.startOffset = this.viewOffset;
                    }
                }
            }

            if (evt.buttons == 4) {
                let over = this.overRobot();

                if (over) {
                    this.rotatingRobot = [over.id, over.color];
                }
            }
        }
    }

    // Stopping the dragging
    mouseUp(evt)
    {
        if (this.draggingRobot && this.mousePos) {
            let robot = robotById(this.draggingRobot[0], this.draggingRobot[1]);
            if (robot) {
                var pos = this.mousePosMeters();

                // XXX: Communicate with the API
                robot.x = pos[0];
                robot.y = pos[1];
            }
        }

        if (this.rotatingRobot && this.mousePos) {
            let robot = robotById(this.rotatingRobot[0], this.rotatingRobot[1]);
            if (robot) {
                var angle = this.mouseAngle(robot);

                // XXX: Communicate with the API
                robot.orientation = angle;
            }
        }

        if (this.draggingBall && this.mousePos) {
            let pos = this.mousePosMeters();

            // XXX: Communicate with the API
            ball = pos;
        }

        this.dragging = false;
        this.draggingRobot = null;
        this.rotatingRobot = null;
        this.draggingBall = false;
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
            console.log(evt);
        }
    }
}

class Manager
{
    constructor() {
        $('.panel-content.not-shown').hide();

        $('.panel-block').each(function() {
            let content = $(this).find('.panel-content');
            var title = $(this).find('.panel-title');

            if (content.is(':visible')) {
                title.find('h4').prepend('<img class="expand-collapse" src="collapse.png"/>');
            } else {
                title.find('h4').prepend('<img class="expand-collapse" src="expand.png"/>');
            }

            title.click(function() {
                if (content.is(':visible')) {
                    content.hide();
                    title.find('h4 .expand-collapse').attr('src', 'expand.png');
                } else {
                    content.show();
                    title.find('h4 .expand-collapse').attr('src', 'collapse.png');
                }
            });
        });

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

        $('.robots .enable-disable').click(function() {
            var id = parseInt($(this).attr('rel'));
            var robot = robotById(id);

            if (robot) {
                robot.enabled = !robot.enabled;

                if (robot.enabled) {
                    $(this).removeClass('btn-success');
                    $(this).addClass('btn-danger');
                    $(this).text('Disable');
                } else {
                    $(this).addClass('btn-success');
                    $(this).removeClass('btn-danger');
                    $(this).text('Enable');
                }
            }
        });
    }
}

$(document).ready(function() {
    // Instantiating the viewer
    var viewer = new Viewer();

    // Panels manager
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
    $('.reset-view').click(function() {
        viewer.resetRatio();
    });
    $('.drag-lock').click(function() {
        viewer.dragLock = !viewer.dragLock;

        if (viewer.dragLock) {
            $(this).text('Unlock the drag');
            $(this).addClass('btn-success');
            $(this).removeClass('btn-danger');
        } else {
            $(this).text('Lock the drag');
            $(this).addClass('btn-danger');
            $(this).removeClass('btn-success');
        }
    });
});
