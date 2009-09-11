///////////////////////////////////////////////////////////////////////////
// C++ code generated with wxFormBuilder (version Apr 21 2008)
// http://www.wxformbuilder.org/
//
// PLEASE DO "NOT" EDIT THIS FILE!
///////////////////////////////////////////////////////////////////////////

#include "rosout_list_control.h"

#include "rosout_generated.h"

///////////////////////////////////////////////////////////////////////////
using namespace rxtools;

RosoutPanelBase::RosoutPanelBase( wxWindow* parent, wxWindowID id, const wxPoint& pos, const wxSize& size, long style ) : wxPanel( parent, id, pos, size, style )
{
	this->SetMinSize( wxSize( 273,138 ) );
	
	wxBoxSizer* bSizer2;
	bSizer2 = new wxBoxSizer( wxVERTICAL );
	
	wxBoxSizer* bSizer3;
	bSizer3 = new wxBoxSizer( wxHORIZONTAL );
	
	wxBoxSizer* bSizer8;
	bSizer8 = new wxBoxSizer( wxHORIZONTAL );
	
	bSizer3->Add( bSizer8, 1, wxEXPAND, 5 );
	
	wxBoxSizer* bSizer4;
	bSizer4 = new wxBoxSizer( wxVERTICAL );
	
	bSizer3->Add( bSizer4, 1, wxEXPAND, 5 );
	
	bSizer2->Add( bSizer3, 0, wxEXPAND, 5 );
	
	wxBoxSizer* bSizer10;
	bSizer10 = new wxBoxSizer( wxVERTICAL );
	
	table_ = new rxtools::RosoutListControl( this, wxID_ANY, wxDefaultPosition, wxDefaultSize, wxLC_HRULES|wxLC_REPORT|wxLC_SINGLE_SEL|wxLC_VIRTUAL|wxLC_VRULES|wxCLIP_CHILDREN );
	bSizer10->Add( table_, 1, wxALL|wxEXPAND, 5 );
	
	wxBoxSizer* bSizer9;
	bSizer9 = new wxBoxSizer( wxHORIZONTAL );
	
	m_staticText1 = new wxStaticText( this, wxID_ANY, wxT("Include:"), wxDefaultPosition, wxDefaultSize, 0 );
	m_staticText1->Wrap( -1 );
	bSizer9->Add( m_staticText1, 0, wxALL|wxALIGN_CENTER_VERTICAL, 5 );
	
	include_text_ = new wxTextCtrl( this, wxID_ANY, wxEmptyString, wxDefaultPosition, wxDefaultSize, 0 );
	bSizer9->Add( include_text_, 1, wxALL|wxEXPAND, 5 );
	
	m_staticText11 = new wxStaticText( this, wxID_ANY, wxT("Exclude:"), wxDefaultPosition, wxDefaultSize, 0 );
	m_staticText11->Wrap( -1 );
	bSizer9->Add( m_staticText11, 0, wxALL|wxALIGN_CENTER_VERTICAL, 5 );
	
	exclude_text_ = new wxTextCtrl( this, wxID_ANY, wxEmptyString, wxDefaultPosition, wxDefaultSize, 0 );
	bSizer9->Add( exclude_text_, 1, wxALL|wxEXPAND, 5 );
	
	regex_checkbox_ = new wxCheckBox( this, wxID_ANY, wxT("Regex"), wxDefaultPosition, wxDefaultSize, 0 );
	
	bSizer9->Add( regex_checkbox_, 0, wxALL|wxALIGN_CENTER_VERTICAL, 5 );
	
	clear_button_ = new wxButton( this, wxID_ANY, wxT("Clear Messages"), wxDefaultPosition, wxDefaultSize, 0 );
	bSizer9->Add( clear_button_, 0, wxALL, 5 );
	
	pause_button_ = new wxToggleButton( this, wxID_ANY, wxT("Pause"), wxDefaultPosition, wxDefaultSize, 0 );
	bSizer9->Add( pause_button_, 0, wxALL|wxALIGN_CENTER_VERTICAL, 5 );
	
	setup_button_ = new wxButton( this, wxID_ANY, wxT("Setup"), wxDefaultPosition, wxDefaultSize, 0 );
	bSizer9->Add( setup_button_, 0, wxALL|wxALIGN_RIGHT, 5 );
	
	bSizer10->Add( bSizer9, 0, wxEXPAND, 5 );
	
	bSizer2->Add( bSizer10, 1, wxEXPAND, 5 );
	
	this->SetSizer( bSizer2 );
	this->Layout();
	
	// Connect Events
	include_text_->Connect( wxEVT_COMMAND_TEXT_UPDATED, wxCommandEventHandler( RosoutPanelBase::onIncludeText ), NULL, this );
	exclude_text_->Connect( wxEVT_COMMAND_TEXT_UPDATED, wxCommandEventHandler( RosoutPanelBase::onExcludeText ), NULL, this );
	regex_checkbox_->Connect( wxEVT_COMMAND_CHECKBOX_CLICKED, wxCommandEventHandler( RosoutPanelBase::onRegexChecked ), NULL, this );
	clear_button_->Connect( wxEVT_COMMAND_BUTTON_CLICKED, wxCommandEventHandler( RosoutPanelBase::onClear ), NULL, this );
	pause_button_->Connect( wxEVT_COMMAND_TOGGLEBUTTON_CLICKED, wxCommandEventHandler( RosoutPanelBase::onPause ), NULL, this );
	setup_button_->Connect( wxEVT_COMMAND_BUTTON_CLICKED, wxCommandEventHandler( RosoutPanelBase::onSetup ), NULL, this );
}

RosoutPanelBase::~RosoutPanelBase()
{
	// Disconnect Events
	include_text_->Disconnect( wxEVT_COMMAND_TEXT_UPDATED, wxCommandEventHandler( RosoutPanelBase::onIncludeText ), NULL, this );
	exclude_text_->Disconnect( wxEVT_COMMAND_TEXT_UPDATED, wxCommandEventHandler( RosoutPanelBase::onExcludeText ), NULL, this );
	regex_checkbox_->Disconnect( wxEVT_COMMAND_CHECKBOX_CLICKED, wxCommandEventHandler( RosoutPanelBase::onRegexChecked ), NULL, this );
	clear_button_->Disconnect( wxEVT_COMMAND_BUTTON_CLICKED, wxCommandEventHandler( RosoutPanelBase::onClear ), NULL, this );
	pause_button_->Disconnect( wxEVT_COMMAND_TOGGLEBUTTON_CLICKED, wxCommandEventHandler( RosoutPanelBase::onPause ), NULL, this );
	setup_button_->Disconnect( wxEVT_COMMAND_BUTTON_CLICKED, wxCommandEventHandler( RosoutPanelBase::onSetup ), NULL, this );
}

RosoutSetupDialogBase::RosoutSetupDialogBase( wxWindow* parent, wxWindowID id, const wxString& title, const wxPoint& pos, const wxSize& size, long style ) : wxDialog( parent, id, title, pos, size, style )
{
	this->SetSizeHints( wxSize( -1,-1 ), wxDefaultSize );
	
	wxBoxSizer* bSizer8;
	bSizer8 = new wxBoxSizer( wxVERTICAL );
	
	wxStaticBoxSizer* sbSizer2;
	sbSizer2 = new wxStaticBoxSizer( new wxStaticBox( this, wxID_ANY, wxT("Rosout Topic") ), wxVERTICAL );
	
	wxBoxSizer* bSizer91;
	bSizer91 = new wxBoxSizer( wxHORIZONTAL );
	
	topic_ = new wxTextCtrl( this, wxID_ANY, wxEmptyString, wxDefaultPosition, wxDefaultSize, wxTE_PROCESS_ENTER );
	bSizer91->Add( topic_, 1, wxALL|wxALIGN_CENTER_VERTICAL|wxEXPAND, 5 );
	
	topic_browse_button_ = new wxButton( this, wxID_ANY, wxT(" ... "), wxDefaultPosition, wxDefaultSize, wxBU_EXACTFIT );
	bSizer91->Add( topic_browse_button_, 0, wxALL|wxALIGN_CENTER_VERTICAL, 5 );
	
	sbSizer2->Add( bSizer91, 1, wxEXPAND, 5 );
	
	bSizer8->Add( sbSizer2, 0, wxEXPAND, 5 );
	
	wxStaticBoxSizer* sbSizer21;
	sbSizer21 = new wxStaticBoxSizer( new wxStaticBox( this, wxID_ANY, wxT("Messages") ), wxVERTICAL );
	
	wxBoxSizer* bSizer10;
	bSizer10 = new wxBoxSizer( wxHORIZONTAL );
	
	m_staticText2 = new wxStaticText( this, wxID_ANY, wxT("Buffer Size"), wxDefaultPosition, wxDefaultSize, 0 );
	m_staticText2->Wrap( -1 );
	bSizer10->Add( m_staticText2, 0, wxALL|wxALIGN_CENTER_VERTICAL, 5 );
	
	buffer_size_spinner_ = new wxSpinCtrl( this, wxID_ANY, wxEmptyString, wxDefaultPosition, wxDefaultSize, wxSP_ARROW_KEYS, 0, 1000000, 20000 );
	bSizer10->Add( buffer_size_spinner_, 0, wxALL, 5 );
	
	sbSizer21->Add( bSizer10, 0, wxALIGN_RIGHT, 5 );
	
	bSizer8->Add( sbSizer21, 1, wxEXPAND, 5 );
	
	wxBoxSizer* bSizer9;
	bSizer9 = new wxBoxSizer( wxHORIZONTAL );
	
	m_sdbSizer1 = new wxStdDialogButtonSizer();
	m_sdbSizer1OK = new wxButton( this, wxID_OK );
	m_sdbSizer1->AddButton( m_sdbSizer1OK );
	m_sdbSizer1Cancel = new wxButton( this, wxID_CANCEL );
	m_sdbSizer1->AddButton( m_sdbSizer1Cancel );
	m_sdbSizer1->Realize();
	bSizer9->Add( m_sdbSizer1, 1, wxEXPAND|wxALL|wxALIGN_BOTTOM, 5 );
	
	bSizer8->Add( bSizer9, 0, wxEXPAND, 5 );
	
	this->SetSizer( bSizer8 );
	this->Layout();
	
	// Connect Events
	topic_browse_button_->Connect( wxEVT_COMMAND_BUTTON_CLICKED, wxCommandEventHandler( RosoutSetupDialogBase::onTopicBrowse ), NULL, this );
	m_sdbSizer1Cancel->Connect( wxEVT_COMMAND_BUTTON_CLICKED, wxCommandEventHandler( RosoutSetupDialogBase::onCancel ), NULL, this );
	m_sdbSizer1OK->Connect( wxEVT_COMMAND_BUTTON_CLICKED, wxCommandEventHandler( RosoutSetupDialogBase::onOk ), NULL, this );
}

RosoutSetupDialogBase::~RosoutSetupDialogBase()
{
	// Disconnect Events
	topic_browse_button_->Disconnect( wxEVT_COMMAND_BUTTON_CLICKED, wxCommandEventHandler( RosoutSetupDialogBase::onTopicBrowse ), NULL, this );
	m_sdbSizer1Cancel->Disconnect( wxEVT_COMMAND_BUTTON_CLICKED, wxCommandEventHandler( RosoutSetupDialogBase::onCancel ), NULL, this );
	m_sdbSizer1OK->Disconnect( wxEVT_COMMAND_BUTTON_CLICKED, wxCommandEventHandler( RosoutSetupDialogBase::onOk ), NULL, this );
}

TextboxDialog::TextboxDialog( wxWindow* parent, wxWindowID id, const wxString& title, const wxPoint& pos, const wxSize& size, long style ) : wxDialog( parent, id, title, pos, size, style )
{
	this->SetSizeHints( wxDefaultSize, wxDefaultSize );
	
	wxBoxSizer* bSizer11;
	bSizer11 = new wxBoxSizer( wxVERTICAL );
	
	text_control_ = new wxTextCtrl( this, wxID_ANY, wxEmptyString, wxDefaultPosition, wxDefaultSize, wxTE_MULTILINE );
	bSizer11->Add( text_control_, 1, wxALL|wxEXPAND, 5 );
	
	this->SetSizer( bSizer11 );
	this->Layout();
}

TextboxDialog::~TextboxDialog()
{
}

LoggerLevelPanelBase::LoggerLevelPanelBase( wxWindow* parent, wxWindowID id, const wxPoint& pos, const wxSize& size, long style ) : wxPanel( parent, id, pos, size, style )
{
	wxBoxSizer* bSizer12;
	bSizer12 = new wxBoxSizer( wxHORIZONTAL );
	
	wxStaticBoxSizer* sbSizer3;
	sbSizer3 = new wxStaticBoxSizer( new wxStaticBox( this, wxID_ANY, wxT("Nodes") ), wxVERTICAL );
	
	nodes_box_ = new wxListBox( this, wxID_ANY, wxDefaultPosition, wxDefaultSize, 0, NULL, 0 ); 
	sbSizer3->Add( nodes_box_, 1, wxALL|wxEXPAND, 5 );
	
	nodes_refresh_ = new wxButton( this, wxID_ANY, wxT("Refresh Nodes"), wxDefaultPosition, wxDefaultSize, 0 );
	sbSizer3->Add( nodes_refresh_, 0, wxALL|wxEXPAND, 5 );
	
	bSizer12->Add( sbSizer3, 1, wxEXPAND, 5 );
	
	wxStaticBoxSizer* sbSizer31;
	sbSizer31 = new wxStaticBoxSizer( new wxStaticBox( this, wxID_ANY, wxT("Loggers") ), wxVERTICAL );
	
	loggers_box_ = new wxListBox( this, wxID_ANY, wxDefaultPosition, wxDefaultSize, 0, NULL, 0 ); 
	sbSizer31->Add( loggers_box_, 1, wxALL|wxEXPAND, 5 );
	
	bSizer12->Add( sbSizer31, 1, wxEXPAND, 5 );
	
	wxStaticBoxSizer* sbSizer311;
	sbSizer311 = new wxStaticBoxSizer( new wxStaticBox( this, wxID_ANY, wxT("Levels") ), wxVERTICAL );
	
	levels_box_ = new wxListBox( this, wxID_ANY, wxDefaultPosition, wxDefaultSize, 0, NULL, 0 ); 
	sbSizer311->Add( levels_box_, 1, wxALL|wxEXPAND, 5 );
	
	bSizer12->Add( sbSizer311, 1, wxEXPAND, 5 );
	
	this->SetSizer( bSizer12 );
	this->Layout();
	
	// Connect Events
	nodes_box_->Connect( wxEVT_COMMAND_LISTBOX_SELECTED, wxCommandEventHandler( LoggerLevelPanelBase::onNodeSelected ), NULL, this );
	nodes_refresh_->Connect( wxEVT_COMMAND_BUTTON_CLICKED, wxCommandEventHandler( LoggerLevelPanelBase::onNodesRefresh ), NULL, this );
	loggers_box_->Connect( wxEVT_COMMAND_LISTBOX_SELECTED, wxCommandEventHandler( LoggerLevelPanelBase::onLoggerSelected ), NULL, this );
	levels_box_->Connect( wxEVT_COMMAND_LISTBOX_SELECTED, wxCommandEventHandler( LoggerLevelPanelBase::onLevelSelected ), NULL, this );
}

LoggerLevelPanelBase::~LoggerLevelPanelBase()
{
	// Disconnect Events
	nodes_box_->Disconnect( wxEVT_COMMAND_LISTBOX_SELECTED, wxCommandEventHandler( LoggerLevelPanelBase::onNodeSelected ), NULL, this );
	nodes_refresh_->Disconnect( wxEVT_COMMAND_BUTTON_CLICKED, wxCommandEventHandler( LoggerLevelPanelBase::onNodesRefresh ), NULL, this );
	loggers_box_->Disconnect( wxEVT_COMMAND_LISTBOX_SELECTED, wxCommandEventHandler( LoggerLevelPanelBase::onLoggerSelected ), NULL, this );
	levels_box_->Disconnect( wxEVT_COMMAND_LISTBOX_SELECTED, wxCommandEventHandler( LoggerLevelPanelBase::onLevelSelected ), NULL, this );
}
