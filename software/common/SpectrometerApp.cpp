/*
    SpectrometerApp.cpp - mainform class for Spectrometer QT application sample

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
#include <QPalette>
#include <QProxyStyle>
#include <QThread>
#include <QStyle>
#include <QStyleFactory>
#include <QString>

#include <math.h>

#include "SpectrometerApp.h"
#include "spectron_cct.h"

#define APP_VERSION " v1.2"

#define MAIN_TITLE APP_NAME APP_VERSION

#define STATE_SECTION "Saved State"

#if defined( Q_OS_MACX )
#define BUNDLE_ID CFSTR("SpectrometerApp")
#if QT_VERSION >= 0x050000
#include <QtPlugin>
Q_IMPORT_PLUGIN (QCocoaIntegrationPlugin);
#endif
#endif

#if defined( Q_OS_WIN )
//#include <omp.h>
#if defined(_QT_STATIC_) && QT_VERSION >= 0x050000
#include <QtPlugin>
Q_IMPORT_PLUGIN (QWindowsIntegrationPlugin);
#endif
#endif

// --------------------------------------------------------
//    static data
// --------------------------------------------------------

// --------------------------------------------------------
//    helper functions
// --------------------------------------------------------

// --------------------------------------------------------
//    SpectrometerApp class
// --------------------------------------------------------
SpectrometerApp::SpectrometerApp(QWidget *parent, Qt::WindowFlags flags)
    : QMainWindow(parent, flags), overrideCursorSet(false),
      m_spectron(), m_pAPI(ParticleAPI::instance()), m_specSeries(0),
      m_axisX(0), m_axisY(0), ignoreUiUpdates(false)
{
    ui.setupUi(this);

    // initial setup
    ui.chkbApplySpResp->setCheckState(
        m_spectron.applySpectralCorrections() ? Qt::Checked : Qt::Unchecked);

    // buttons
    connect(ui.btnLogin, SIGNAL(clicked()), this, SLOT(login()));
    connect(ui.btnMeasure, SIGNAL(clicked()), this, SLOT(measure()));
    connect(ui.btnMeasureBlack, SIGNAL(clicked()), this, SLOT(measureBlack()));
    connect(ui.btnMeasSat, SIGNAL(clicked()), this, SLOT(measureSaturation()));
    connect(ui.btnMeasMinBlack, SIGNAL(clicked()), this, SLOT(measureMinBlack()));
    connect(ui.btnCalibrSpectral, SIGNAL(clicked()), this, SLOT(calibrateSpectralResponse()));
    connect(ui.btnResetSpectralCal, SIGNAL(clicked()), this, SLOT(resetSpectralResponse()));
    connect(ui.btnCalcT, SIGNAL(clicked()), this, SLOT(calculateLampTemperature()));
    connect(ui.btnSetRange, SIGNAL(clicked()), this, SLOT(setSpectralRange()));

    // comboboxes
    connect(ui.cboxAdcRef, SIGNAL(currentIndexChanged(int)), this, SLOT(setADCRef(int)));
    connect(ui.cboxGain, SIGNAL(currentIndexChanged(int)), this, SLOT(setGain(int)));
    connect(ui.cboxUnits, SIGNAL(currentIndexChanged(int)), this, SLOT(setUnits(int)));
    connect(ui.cboxMeasResultType, SIGNAL(currentIndexChanged(int)), this, SLOT(setMeasResultType(int)));
    connect(ui.cboxMeasType, SIGNAL(currentIndexChanged(int)), this, SLOT(setMeasType(int)));
    connect(ui.cbxSpRangeType, SIGNAL(currentIndexChanged(int)), this, SLOT(spectralRespTypeChanged(int)));

    // checkboxes
    connect(ui.chkbApplySpResp, SIGNAL(stateChanged(int)), this, SLOT(applySpectralResponseChanged(int)));

    // tab changes
    connect(ui.tabs, SIGNAL(currentChanged(int)), this, SLOT(tabChanged(int)));

    setWindowTitle(MAIN_TITLE);

    // setup chart
    ui.wChart->chart()->setTheme(QChart::ChartThemeDark);
    ui.wChart->chart()->legend()->hide();

    m_specSeries = new QLineSeries();
	m_specSeries->clear();
    m_specSeries->append(340,0);
    m_specSeries->append(850,0);
    ui.wChart->chart()->addSeries(m_specSeries);
    m_axisX = new QValueAxis();
    m_axisY = new QValueAxis();
    m_axisX->setRange(340.0,850.0);
    m_axisX->setTickCount((850-340)/30+1);
    m_axisX->setLabelFormat("%d");
    m_axisY->setRange(0, 1.0);
    m_axisY->setTickCount(11);
    ui.wChart->chart()->setAxisX(m_axisX, m_specSeries);
    ui.wChart->chart()->setAxisY(m_axisY, m_specSeries);
}


SpectrometerApp::~SpectrometerApp()
{
}
void SpectrometerApp::updateAxis()
{
    double maxVal = ceil(105*m_spectron.getMaxLastMeasuredValue())/100;
    int ticks = 11;
    
    if (maxVal==0.0)
    {
        maxVal = 1.0;
        if (m_spectron.getMeasureType() == SpectronDevice::MEASURE_VOLTAGE)
        {
            maxVal = ceil(m_spectron.getSatVoltage()/0.5)*0.5;
            ticks = maxVal/0.25 + 1;
        }
    }

    if (m_axisY->max() != maxVal)
    {
        m_specSeries->clear();
        m_specSeries->append(m_spectron.getMinWavelength(),0);
        m_specSeries->append(m_spectron.getMaxWavelength(),0);
        m_axisY->setRange(0, maxVal);
        m_axisY->setTickCount(ticks);
    }
}

void SpectrometerApp::updateWidgets()
{
    QString title(MAIN_TITLE);

    setWindowTitle(title);

    if (!m_spectron.isConnected())
        return;

    if (m_spectron.supportsGain())
        ui.lblHGSval->setText(
            QString("%1 V").arg(
                m_spectron.getSatVoltage(SpectronDevice::HIGH_GAIN), 0, 'f', 2));
    ui.lblNGSval->setText(
        QString("%1 V").arg(
            m_spectron.getSatVoltage(SpectronDevice::NO_GAIN), 0, 'f', 2));
    ui.lblMinBval->setText(
        QString("%1 V").arg(
            m_spectron.getMinBlackVoltage(), 0, 'f', 2));
}

void SpectrometerApp::updateRanges()
{
    if (!m_spectron.isConnected())
        return;

    // update ranges
    int minWv = m_spectron.getMinWavelength();
    int maxWv = m_spectron.getMaxWavelength();
    minWv = ((minWv+3)/5)*5;  // round up to nearest 5
    maxWv = (maxWv/5)*5;        // round down to nearest 5
    ui.lblRange->setText(QString("Spectral Range: %1 - %2 nm").arg(minWv).arg(maxWv));
    ui.spbMinWv->setValue(minWv);
    ui.spbMaxWv->setValue(maxWv);
    
    // update the chart
    m_axisX->setRange(minWv, maxWv);
    m_axisX->setTickCount(15);
}

void SpectrometerApp::setADCRef(int idx)
{
    if (ignoreUiUpdates)
        return;

    setOverrideCursor(QCursor(Qt::WaitCursor));

    if (m_spectron.isConnected())
        m_spectron.setADCReference((SpectronDevice::TAdcRef)idx);
    restoreOverrideCursor();
}

void SpectrometerApp::setGain(int idx)
{
    if (ignoreUiUpdates)
        return;

    setOverrideCursor(QCursor(Qt::WaitCursor));
    if (m_spectron.isConnected())
    {
        m_spectron.setGain((SpectronDevice::TGain)idx);
        updateAxis();
    }

    restoreOverrideCursor();
}

void SpectrometerApp::setUnits(int idx)
{
    if (ignoreUiUpdates)
        return;

    setOverrideCursor(QCursor(Qt::WaitCursor));
    if (m_spectron.isConnected())
        if (ui.cboxUnits->currentIndex() == 1)
            ui.spbIntegration->setValue(m_spectron.getIntegTime()/1000);
        else
            ui.spbIntegration->setValue(m_spectron.getIntegTime());
    restoreOverrideCursor();
}

void SpectrometerApp::setMeasResultType(int idx)
{
    if (ignoreUiUpdates)
        return;

    setOverrideCursor(QCursor(Qt::WaitCursor));
    if (m_spectron.isConnected())
    {
        m_spectron.setMeasureType((SpectronDevice::TMeasType)idx);
        updateAxis();
    }

    restoreOverrideCursor();
}

void SpectrometerApp::setMeasType(int idx)
{
    if (ignoreUiUpdates)
        return;

    bool intEnable = idx==0;

    ui.lblIntegration->setEnabled(intEnable);
    ui.spbIntegration->setEnabled(intEnable);
    
    if (!m_spectron.supportsGain())
    {
        ui.lblMeasureTime->setEnabled(intEnable);
        ui.spbMeasureTime->setEnabled(intEnable);
    }
}

void SpectrometerApp::tabChanged(int idx)
{
    setOverrideCursor(QCursor(Qt::WaitCursor));
    if (idx == 1)
    {
        // load up the normalisation data
        if (m_spectron.isConnected() &&
            m_spectron.getSpectrometerData(SpectronDevice::ET_NORMALISATION))
            readMeasurement(false);
    }
    else
    {
        // load up the measurement data
        if (m_spectron.isConnected() &&
            m_spectron.getSpectrometerData(SpectronDevice::ET_MEASUREMENT))
            readMeasurement(true);
    }
    restoreOverrideCursor();
}

void SpectrometerApp::login()
{
    setOverrideCursor(QCursor(Qt::WaitCursor));
    // login with user/password - get non expiring auth token.
    if (m_pAPI.login(ui.edtUser->text(), ui.edtPwd->text(), 0))
    {
        TParticleDeviceList devices;
        if (m_pAPI.getAllMatchingDevices(devices, QString(""), true)
            && !devices.empty())
        {
            // iterate the connected device list and find internal
            // devices for spectrometer
            TParticleDeviceList::iterator it = devices.begin();
            while (it != devices.end())
            {
                if (it->refresh())
                {
                    QString boardType = it->getVariableValue("BOARD_TYPE").toString();
                    if (boardType == QString("SPEC2_SPECTROMETER"))
                    {
                        // spectrometer device
                        m_spectron = *it;
                        m_spectron.refresh();
                        ignoreUiUpdates = true;
                        ui.cboxAdcRef->setCurrentIndex(m_spectron.getADCReference());
                        ui.cboxMeasResultType->setCurrentIndex(m_spectron.getMeasureType());
                        if (m_spectron.supportsGain())
                        {
                            ui.lblGain->setEnabled(true);
                            ui.cboxGain->setEnabled(true);
                            ui.lblHGStxt->setEnabled(true);
                            ui.lblHGSval->setEnabled(true);
                            ui.cboxGain->setCurrentIndex(m_spectron.getGain());
                            ui.lblMeasureTime->setEnabled(false);
                            ui.spbMeasureTime->setEnabled(false);
                            ui.lblSpectrometer->setText("Spectrometer: Hamamatsu C12666MA  ");
                        }
                        else
                        {
                            ui.lblGain->setEnabled(false);
                            ui.cboxGain->setEnabled(false);
                            ui.lblHGStxt->setEnabled(false);
                            ui.lblHGSval->setEnabled(false);
                            ui.lblMeasureTime->setEnabled(true);
                            ui.spbMeasureTime->setEnabled(true);
                            ui.lblSpectrometer->setText("Spectrometer: Hamamatsu C12880MA  ");
                        }
                        if (ui.cboxUnits->currentIndex() == 1)
                            ui.spbIntegration->setValue(m_spectron.getIntegTime()/1000);
                        else
                            ui.spbIntegration->setValue(m_spectron.getIntegTime());

                        // update data
                        updateWidgets();
                        updateRanges();
                        tabChanged(ui.tabs->currentIndex());

                        ignoreUiUpdates = false;
                    }
                }
                ++it;
            }

        }
    }
    else
        ui.txtCSV->setPlainText(m_pAPI.getLastError());

    restoreOverrideCursor();
}

void SpectrometerApp::measure()
{
    setOverrideCursor(QCursor(Qt::WaitCursor));
    if (m_spectron.isConnected())
    {
        if (ui.cboxMeasType->currentIndex() == 0)
        {
            int integTime = ui.spbIntegration->value();
            int setIntegTime = m_spectron.getIntegTime();

            if (ui.cboxUnits->currentIndex() == 1)
            {
                integTime *= 1000;
                setIntegTime /= 1000;
            }

            // set integration time if changed
            if (setIntegTime != ui.spbIntegration->value())
                m_spectron.setIntegrationTime(integTime);

            int measureTime = 0;
            if (!m_spectron.supportsGain())
                measureTime = ui.spbMeasureTime->value()*1000;

           if (m_spectron.measure(measureTime))
                readMeasurement(true);
            else
                ui.txtCSV->setPlainText(m_pAPI.getLastError());
        }
        else
        {
            if (m_spectron.measureAuto(
                  (SpectronDevice::TAutoType)(ui.cboxMeasType->currentIndex()-1)))
            {
                ui.cboxAdcRef->setCurrentIndex(m_spectron.getADCReference());
                ui.cboxGain->setCurrentIndex(m_spectron.getGain());
                if (ui.cboxUnits->currentIndex() == 1)
                    ui.spbIntegration->setValue(m_spectron.getIntegTime()/1000);
                else
                    ui.spbIntegration->setValue(m_spectron.getIntegTime());
                readMeasurement(true);
            }
            else
                ui.txtCSV->setPlainText(m_pAPI.getLastError());
        }
    }

    restoreOverrideCursor();
}

void SpectrometerApp::measureBlack()
{
    setOverrideCursor(QCursor(Qt::WaitCursor));
    if (m_spectron.isConnected())
    {
        int integTime = ui.spbIntegration->value();
        int setIntegTime = m_spectron.getIntegTime();

        if (ui.cboxUnits->currentIndex() == 1)
        {
            integTime *= 1000;
            setIntegTime /= 1000;
        }

        if (setIntegTime != ui.spbIntegration->value())
            m_spectron.setIntegrationTime(integTime);

        int measureTime = 0;
        if (!m_spectron.supportsGain())
            measureTime = ui.spbMeasureTime->value()*1000;

       if (m_spectron.measureBlack(measureTime))
       {
            m_spectron.getSpectrometerData(SpectronDevice::ET_MEASUREMENT);
            readMeasurement(true);
        }
        else
            ui.txtCSV->setPlainText(m_pAPI.getLastError());
    }

    restoreOverrideCursor();
}

void SpectrometerApp::measureSaturation()
{
    if (ignoreUiUpdates || !m_spectron.isConnected())
        return;

    int dlgRes = showMessage(
                    tr("Confirmation"),
                    tr("For spectrometer saturation calibration, you need to\n "
                       "expose spectrometer to the bright light source (lamp)!"),
                    tr("Please confirm this is done if you want\n"
                        "to go ahead with calibration?"),
                    QMessageBox::Question,
                    QMessageBox::Ok | QMessageBox::Cancel);

    if (dlgRes == QMessageBox::Ok)
    {
        setOverrideCursor(QCursor(Qt::WaitCursor));
        m_spectron.measureSaturation();
        restoreOverrideCursor();
        updateWidgets();
    }
}

void SpectrometerApp::measureMinBlack()
{
    if (ignoreUiUpdates || !m_spectron.isConnected())
        return;

    int dlgRes = showMessage(
                    tr("Confirmation"),
                    tr("For minimal black calibration, you need to cover\n"
                       "spectrometer entrance before proceeding!"),
                    tr("Please confirm this is done if you want\n"
                        "to go ahead with calibration?"),
                    QMessageBox::Question,
                    QMessageBox::Ok | QMessageBox::Cancel);

    if (dlgRes == QMessageBox::Ok)
    {
        setOverrideCursor(QCursor(Qt::WaitCursor));
        m_spectron.setMinBlack();
        restoreOverrideCursor();
        updateWidgets();
    }
}

void SpectrometerApp::resetSpectralResponse()
{
    if (ignoreUiUpdates || !m_spectron.isConnected())
        return;

    int dlgRes = showMessage(
                    tr("Reset"),
                    tr("This is going to reset previous calibration\n"
                       "of the spectral response!"),
                    tr("Please confirm that you want to proceed?"),
                    QMessageBox::Question,
                    QMessageBox::Ok | QMessageBox::Cancel);

    if (dlgRes == QMessageBox::Cancel)
        return;

    setOverrideCursor(QCursor(Qt::WaitCursor));
    m_spectron.calibrateSpectralResponse(0.0);
    restoreOverrideCursor();
}

void SpectrometerApp::calibrateSpectralResponse()
{
    if (ignoreUiUpdates || !m_spectron.isConnected())
        return;

    double lampT = ui.spbT->value();

    if (lampT < 1700)
    {
        showMessage(tr("Error"), tr("Lamp temperature is not set!"));
        return;
    }

    int dlgRes = showMessage(
                    tr("Step 1"),
                    tr("Position the lamp such that spectrometer reads\n"
                       "reflected or diffused light before proceeding!"),
                    tr("Please confirm this is done if you want\n"
                        "to go ahead with calibration?"),
                    QMessageBox::Question,
                    QMessageBox::Ok | QMessageBox::Cancel);

    if (dlgRes == QMessageBox::Cancel)
        return;

    // step 1 - establish measurement parameters
    setOverrideCursor(QCursor(Qt::WaitCursor));
    bool success = m_spectron.measureAuto(SpectronDevice::AUTO_ALL_MAX_RANGE);
    restoreOverrideCursor();
    if (!success)
    {
        showMessage(tr("Error"), tr("Auto measurement failed!"));
        return;
    }

    dlgRes = showMessage(
                    tr("Step 2"),
                    tr("Cover the spectrometer entrance to measure\n"
                       "black levels with established parameters!"),
                    tr("Please confirm this is done if you want\n"
                        "to go ahead with calibration?"),
                    QMessageBox::Question,
                    QMessageBox::Ok | QMessageBox::Cancel);

    if (dlgRes == QMessageBox::Cancel)
        return;

    // step 2 - measure black levels
    setOverrideCursor(QCursor(Qt::WaitCursor));
    success = m_spectron.measureBlack();
    restoreOverrideCursor();
    if (!success)
    {
        showMessage(tr("Error"), tr("Black measurement failed!"));
        return;
    }

    // step 3 - calculate normalisation
    setOverrideCursor(QCursor(Qt::WaitCursor));
    success = m_spectron.calibrateSpectralResponse(lampT);
    restoreOverrideCursor();
    if (!success)
    {
        showMessage(tr("Error"), tr("Normalisation failed!"));
        return;
    }
    readMeasurement(false);
}

void SpectrometerApp::calculateLampTemperature()
{
    double RT0 = ui.spbR0->value();
    double T0 = ui.spbT0->value(); // in Celsius
    double lampI = ui.spbLampI->value();
    double lampV = ui.spbLampV->value();

    if (RT0 <= 0 || T0 <= 0 || lampI <= 0 || lampV <= 0)
    {
        showMessage(tr("Error"), tr("One of the lamp parameters is not set!"));
        return;
    }

    // Calculate working lamp filament temperature Tf(K) from given
    // room temperature, resistance at room tempearture, woring lamp
    // current and voltage. For more details see O. Harang, M. J. Kosch
    // "Absolute Optical Calibrations Using a Simple Tungsten Bulb:Theory"
    T0 += 273.15; // translate to K
    double rT0 = 0.00000125*T0*T0 + 0.0236*T0 - 1.57;
    double rTf = (rT0*lampV)/(lampI*RT0);
    double Tf = 5000*(sqrt(716*rTf+72023)-264)/179;

    ui.spbT->setValue(Tf);
}

void SpectrometerApp::applySpectralResponseChanged(int state)
{
    if (ignoreUiUpdates)
        return;

    m_spectron.setSpectralRespCorrection(state==Qt::Checked);

    if (m_spectron.isConnected() &&
        m_spectron.getSpectrometerData(SpectronDevice::ET_MEASUREMENT))
        readMeasurement(state==Qt::Checked);
}

void SpectrometerApp::setSpectralRange()
{
    if (ignoreUiUpdates || !m_spectron.isConnected())
        return;

    int dlgRes = showMessage(
                    tr("Confirmation"),
                    tr("Setting new spectral range will invalidate\n"
                       "and reset spectral response calibration!"),
                    tr("Please confirm if you want to go ahead?"),
                    QMessageBox::Question,
                    QMessageBox::Ok | QMessageBox::Cancel);

    if (dlgRes == QMessageBox::Ok)
    {
        setOverrideCursor(QCursor(Qt::WaitCursor));
        m_spectron.setSpectralRange(
            (SpectronDevice::TRangeType)ui.cbxSpRangeType->currentIndex(), 
            ui.spbMinWv->value(), 
            ui.spbMaxWv->value());
        restoreOverrideCursor();
        updateRanges();
    }
}

void SpectrometerApp::spectralRespTypeChanged(int idx)
{
    ui.spbMinWv->setEnabled(idx==0);
    ui.spbMaxWv->setEnabled(idx==0);
}

void SpectrometerApp::saveCSV()
{
}

void SpectrometerApp::readMeasurement(bool doColourData, bool csvOnly)
{
    setOverrideCursor(QCursor(Qt::WaitCursor));
    if (m_spectron.isConnected())
    {
        updateAxis();
        
        // populate last measurement
        if (!csvOnly)
            m_specSeries->clear();
        QString csv = "Wavelength,Measurement\n";
        double maxVal = 0, minVal = 1;
        int maxIdx = 0;
        for (int i=0; i<m_spectron.totalPixels(); i++)
        {
            csv.append(QString("%1,%2\n").arg(m_spectron.getWavelength(i)).arg(m_spectron.getLastMeasurement(i)));
            if (m_spectron.getLastMeasurement(i) > maxVal)
            {
                maxVal = m_spectron.getLastMeasurement(i);
                maxIdx = i;
            }
            else if (m_spectron.getLastMeasurement(i) < minVal)
                minVal = m_spectron.getLastMeasurement(i);
            if (!csvOnly)
                m_specSeries->append(m_spectron.getWavelength(i), m_spectron.getLastMeasurement(i));
        }
        if (!csvOnly)
        {
            double CCT = 0, x = 0, y = 0;
            
            ui.lblMaxValue->setText(QString("%1 at %2 nm    ")
                .arg(maxVal, 0, 'F', 4)
                .arg(m_spectron.getWavelength(maxIdx), 0, 'F', 2));

            // calculate and set CCT, x and y
            if (doColourData)
                calculateColourParam(m_spectron, CCT, x, y);
            ui.lblCCT->setText(QString("%1 K    ").arg(CCT, 0, 'F', 0));
            ui.lblX->setText(QString("%1    ").arg(x, 0, 'F', 6));
            ui.lblY->setText(QString("%1    ").arg(y, 0, 'F', 6));
        }

        ui.txtCSV->setPlainText(csv);
    }

    restoreOverrideCursor();
}


void SpectrometerApp::closeEvent(QCloseEvent *event)
{
}

int SpectrometerApp::showMessage(const QString& title,
                          const QString& msgText,
                          const QString& informativeText,
                          QMessageBox::Icon icon,
                          QMessageBox::StandardButtons buttons,
                          QMessageBox::StandardButton defButton)
{
    restoreOverrideCursor();

    QMessageBox msgBox(icon,
                       title,
                       msgText,
                       buttons);
    msgBox.setInformativeText(informativeText);
    msgBox.setDefaultButton(defButton);

    return msgBox.exec();
}

void SpectrometerApp::resizeEvent(QResizeEvent *event)
{
    QMainWindow::resizeEvent(event);
}


// -------------------------------------------------------------------------
//   Event slots
// -------------------------------------------------------------------------
// -------------------------------------------------------------------------
//   Fusion proxy style to disable stupid QStyle::SH_ComboBox_Popup
// -------------------------------------------------------------------------
class DCSProxyStyle : public QProxyStyle
{
public:
    DCSProxyStyle(QStyle *style): QProxyStyle(style) {}

    int styleHint(StyleHint hint, const QStyleOption *option, const QWidget *widget, QStyleHintReturn *returnData) const
    {
        if (hint == QStyle::SH_ComboBox_Popup)
        {
            return 0;
        }
        return QProxyStyle::styleHint(hint, option, widget, returnData);
    }

    void polish (QWidget *w)
    {
#ifdef Q_OS_MACX
        QMenu* mn = qobject_cast<QMenu*>(w);
        if (!mn && !w->testAttribute(Qt::WA_MacNormalSize))
            w->setAttribute(Qt::WA_MacSmallSize);
#endif
    }
};

// -------------------------------------------------------------------------
int main(int argc, char *argv[])
{
    QApplication app(argc, argv);

#if QT_VERSION >= 0x050000
	app.setStyle(new DCSProxyStyle(QStyleFactory::create("fusion")));
#endif

    QPalette palette;
    palette.setColor(QPalette::Window, QColor(83,83,83));
    palette.setColor(QPalette::WindowText, Qt::white);
    palette.setColor(QPalette::Base, QColor(63,63,63));
    palette.setColor(QPalette::AlternateBase, QColor(83,83,83));
    palette.setColor(QPalette::ToolTipBase, QColor(94,180,255));
    palette.setColor(QPalette::ToolTipText, Qt::black);
    palette.setColor(QPalette::Text, Qt::white);
    palette.setColor(QPalette::Button, QColor(83,83,83));
    palette.setColor(QPalette::ButtonText, Qt::white);
    palette.setColor(QPalette::BrightText, Qt::red);
    palette.setColor(QPalette::Highlight, QColor(51,153,255));
    palette.setColor(QPalette::HighlightedText, Qt::black);

    palette.setColor(QPalette::Disabled, QPalette::WindowText, Qt::gray);
    palette.setColor(QPalette::Disabled, QPalette::ButtonText, Qt::gray);

    app.setPalette(palette);

    SpectrometerApp specMain;
    specMain.show();
    return app.exec();
}
