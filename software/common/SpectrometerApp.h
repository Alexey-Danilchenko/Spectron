/*
    SpectrometerApp.h - mainform class for Spectrometer application sample

    Copyright 2017-2019 Alexey Danilchenko

    This program is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation; either version 3, or (at your option)
    any later version with ADDITION (see below).

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program; if not, write to the Free Software
    Foundation, 51 Franklin Street - Fifth Floor, Boston,
    MA 02110-1301, USA.
*/
#ifndef SPECTROMETER_APP_H
#define SPECTROMETER_APP_H

#include <QMainWindow>
#include <QMessageBox>
#include <QLabel>
#include <QString>
#include <QtCharts>

#include "spectron_api.h"

#include "ui_SpectrometerApp.h"

#define APP_NAME "Spectrometer"

using namespace QtCharts;

// --------------------------------------------------------
//    SpectrometerApp class
// --------------------------------------------------------
class SpectrometerApp : public QMainWindow
{
	Q_OBJECT

    // member variables
    Ui::SpectrometerApp ui;

    SpectronDevice m_spectron;
    ParticleAPI& m_pAPI;

    QLineSeries* m_specSeries;
    QValueAxis* m_axisX;
    QValueAxis* m_axisY;

    bool overrideCursorSet;
    bool ignoreUiUpdates;

public:
	SpectrometerApp(QWidget *parent = 0, Qt::WindowFlags flags = 0);
	~SpectrometerApp();

private:

    void closeEvent(QCloseEvent *event);
    void resizeEvent(QResizeEvent *event);

    int showMessage(const QString& title,
                    const QString& msgText,
                    const QString& informativeText=tr(""),
                    QMessageBox::Icon icon=QMessageBox::Critical,
                    QMessageBox::StandardButtons buttons = QMessageBox::NoButton,
                    QMessageBox::StandardButton defButton = QMessageBox::NoButton);

    void setOverrideCursor(const QCursor& cursor)
    {
        if (!overrideCursorSet)
        {
            overrideCursorSet = true;
            QApplication::setOverrideCursor(cursor);
            QApplication::processEvents();
        }
    }

    void restoreOverrideCursor()
    {
        if (overrideCursorSet)
        {
            overrideCursorSet = false;
            QApplication::restoreOverrideCursor();
        }
    }

    void updateAxis();
    void updateStats();
    void updateWidgets();
    void updateRanges();
    void readMeasurement(bool doColourData, bool csvOnly=false);
    
private slots:

    void setADCRef(int idx);
    void setGain(int idx);
    void setUnits(int idx);
    void setMeasResultType(int idx);
    void setMeasType(int idx);
    void tabChanged(int idx);
    void spectralRespTypeChanged(int idx);
    void applySpectralResponseChanged(int state);

    void login();
    void measure();
    void measureBlack();
    void measureSaturation();
    void measureMinBlack();
    void calibrateSpectralResponse();
    void resetSpectralResponse();
    void calculateLampTemperature();
    void setSpectralRange();
    void saveCSV();
};

#endif // SPECTROMETER_APP_H
