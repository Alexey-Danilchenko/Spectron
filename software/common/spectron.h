/*
    spectron.h - mainform class for Spectron application
    
    Copyright 2017-2018 Alexey Danilchenko

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
#ifndef SPECTRON_H
#define SPECTRON_H

#include <QMainWindow>
#include <QMessageBox>
#include <QLabel>
#include <QString>
#include <QtCharts>

#include "spectron_api.h"
#include "motor_api.h"
#include "light_source_api.h"
#include "meas_session.h"

#include "ui_spectron.h"

#define APP_NAME "Spectron"

using namespace QtCharts;

// --------------------------------------------------------
//    Spectron class
// --------------------------------------------------------
class Spectron : public QMainWindow
{
	Q_OBJECT
    
    // member variables
    Ui::Spectron ui;

    SpectronDevice m_spectron;
    SpecMotorDevice m_motor;
    SpecLightSourceDevice *m_lightSource;
    ParticleAPI& m_pAPI;

    QLineSeries* m_specSeries;
    QLineSeries* m_gaussSeries;
    bool unsavedChanges;
    bool overrideCursorSet;
    bool ignoreUiUpdates;
    
public:
	Spectron(QWidget *parent = 0, Qt::WindowFlags flags = 0);
	~Spectron();

private:

    void closeEvent(QCloseEvent *event);
    void resizeEvent(QResizeEvent *event);

    int showMessage(const QString& title,
                    const QString& msgText,
                    const QString& informativeText=tr(""),
                    QMessageBox::Icon icon=QMessageBox::Critical, 
                    QMessageBox::StandardButtons buttons = QMessageBox::NoButton, 
                    QMessageBox::StandardButton defButton = QMessageBox::NoButton);
                    
    void setOverrideCursor(const QCursor& cursor) { 
        if (!overrideCursorSet)
        {
            overrideCursorSet = true;
            QApplication::setOverrideCursor(cursor);
            QApplication::processEvents();
        }
    }

    void restoreOverrideCursor() { 
        if (overrideCursorSet)
        {
            overrideCursorSet = false;
            QApplication::restoreOverrideCursor();
        }
    } 
                     
    //void resizeEvent(QResizeEvent *event);
    void updateStats();
    void updateWidgets();
    bool checkUnsavedAndSave();
    
    void readMeasurement();
    void init();

private slots:

    void setADCRef(int idx);
    void setGain(int idx);
    
    void login();
    void measure();
    void measureAuto(SpectronDevice::TAutoType autoType);
    void measureBlack();
    void measureSpectrum();
    void setWavelength();
    
    void help();
    void about();

    void updateStatus();
};

#endif // SPECTRON_H
